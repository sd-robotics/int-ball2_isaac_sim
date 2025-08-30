
#include "ib2_nav/nav.h"

#include <limits>

namespace
{
    /** 単位変換係数DEG->RAD */
    const double DEG2RAD(M_PI / 180.0);

    /** 単位変換係数RAD->DEG */
    const double RAD2DEG(180.0 / M_PI);

    /** 微小値 */
    const double EPS(1.0E-10);

    /** ISS座標系 */
    const std::string FRAME_ISS("iss_body");

}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
ib2::Nav::Nav(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
    rclcpp::Node("nav", options),
    tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
    tnav_offset_(0,0)
{
    // Get Parameters
    if(getParameter() != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load parameters");
        return;
    }

    // Open CSV File to include errors if specified
    openCSVFile();

    // Create a Navigation topic, and publish it.
    pub_nav_                   = this->create_publisher<ib2_interfaces::msg::Navigation>(
        "/sensor_fusion/navigation", 1);
    pub_sensor_fusion_status_  = this->create_publisher<ib2_interfaces::msg::NavigationStatus>(
        "/sensor_fusion/navigation_status", 1);

    // Create a Attitude topic, and publish it.
    pub_att_                   = this->create_publisher<ib2_interfaces::msg::Attitude>(
        "/nav/attitude", 1);

    // Create a True Navigation topic, and publish it.
    pub_true_nav_              = this->create_publisher<ib2_interfaces::msg::Navigation>(
        "/nav/true/navigation", 1);

    // Create a True Attitude topic, and publish it.
    pub_true_att_              = this->create_publisher<ib2_interfaces::msg::Attitude>(
        "/nav/true/attitude", 1);

    // Subscribers
    sub_odom_         = create_subscription<nav_msgs::msg::Odometry>(
        "/ground_truth", 5, std::bind(&ib2::Nav::odomCallback, this, std::placeholders::_1));
    sub_imu_          = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/imu", 5, std::bind(&ib2::Nav::imuCallback, this, std::placeholders::_1));
    sub_status_       = this->create_subscription<example_interfaces::msg::Int32>(
        "/nav/status", 5, std::bind(&ib2::Nav::statusCallback, this, std::placeholders::_1));       // CHECK: publisher existence
    sub_time_offset_  = this->create_subscription<example_interfaces::msg::Float64>(
        "/nav/time_offset", 5, std::bind(&ib2::Nav::offsetCallback, this, std::placeholders::_1));  // CHECK: publisher existence

    // Nav Parameter Update Server
    nav_param_server_         = this->create_service<ib2_interfaces::srv::UpdateParameter>(
        "/sim/nav/update_params", std::bind(&ib2::Nav::updateParameter, this, std::placeholders::_1, std::placeholders::_2));

    // Nav Marker Correction Server
    marker_correction_server_ = this->create_service<ib2_interfaces::srv::MarkerCorrection>(
        "/sensor_fusion/marker_correction", std::bind(&ib2::Nav::markerCorrection, this, std::placeholders::_1, std::placeholders::_2));

    // Nav ON/OFF Service Server
    switch_power_server_      = this->create_service<ib2_interfaces::srv::SwitchPower>(
        "/nav/switch_power", std::bind(&ib2::Nav::switchPower, this, std::placeholders::_1, std::placeholders::_2));

    // Nav ON/OFF Action Server
    navigation_start_up_ = rclcpp_action::create_server<NavigationStartUp>(
        this, "/sensor_fusion/navigation_start_up",
        std::bind(&ib2::Nav::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ib2::Nav::handleCancel, this, std::placeholders::_1),
        std::bind(&ib2::Nav::handleAccepted, this, std::placeholders::_1));


    // Get Control Duration[s]
    double cnt_duration = controlFreqFluctuation();

    // Timer for callback
    timer_group_                = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_                      = this->create_wall_timer(
        std::chrono::duration<double>(cnt_duration),
        std::bind(&ib2::Nav::navCallBack, this), timer_group_);
    timer_sensor_fusion_status_ = this->create_wall_timer(
        std::chrono::duration<double>(cnt_duration),
        std::bind(&ib2::Nav::sensorFusionStatusCallBack, this), timer_group_);

    // Initialize state
    status_      = initial_nav_on_ ?
        ib2_interfaces::msg::NavigationStatus::NAV_FUSION :
        ib2_interfaces::msg::NavigationStatus::NAV_OFF;
    invalid_nav_ = false;

}

//------------------------------------------------------------------------------
// デストラクタ.
ib2::Nav::~Nav()
{
    if(ifs_.is_open())
    {
        ifs_.close();
    }
}

//------------------------------------------------------------------------------
// ROS Timerのコールバック関数
void ib2::Nav::navCallBack()
{
    // Get True Navigation in docking station coordination frame
    ib2_interfaces::msg::Navigation nav_msgs_h = getTrueNavigation();

    // Transform World Coordination to Home Coordination
    // Not needed as we are already in the home (DS) coordination frame
    // ib2_interfaces::msg::Navigation nav_msgs_h = transformWtoH(nav_msgs_w);

    // Publish True Navigation Message
    pub_true_nav_->publish(nav_msgs_h);

    // Publish True Attitude Message
    pub_true_att_->publish(makeAttMsgFromNavMsg(nav_msgs_h));

    // Add Error
    if (add_error_)
    {
        nav_msgs_h = addError(nav_msgs_h);
        if (invalid_nav_)
        {
            nav_msgs_h.pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();
        }

        // Navigation Delay Model
        nav_buffer_.push(nav_msgs_h);
        int size = static_cast<int>(bias_cnt_ * delay_ + 0.5 + EPS);  // size will never be < 0
        if(nav_buffer_.size() >= (size_t)(size + 1))
        {
            RCLCPP_DEBUG(this->get_logger(), "Navigation Delay is %f[s]",
                        (this->now() - nav_buffer_.front().pose.header.stamp).seconds());

            // Publish Navigation Message
            pub_nav_->publish(nav_buffer_.front());
            // Publish Attitude Message
            pub_att_->publish(makeAttMsgFromNavMsg(nav_buffer_.front()));

            nav_buffer_.pop();
        }
    } else {
        // Publish Navigation Message
        pub_nav_->publish(nav_msgs_h);
        // Publish Attitude Message
        pub_att_->publish(makeAttMsgFromNavMsg(nav_msgs_h));
    }

    // Get Control Duration[s]
    double cnt_duration = controlFreqFluctuation();

    // Cancel previous timer
    timer_->cancel();
    
    // Timer for callback
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(cnt_duration),
        std::bind(&ib2::Nav::navCallBack, this), timer_group_);
}

//------------------------------------------------------------------------------
// ROS Timerのコールバック関数
void ib2::Nav::sensorFusionStatusCallBack()
{
    // Publish Navigation Status Message
    ib2_interfaces::msg::NavigationStatus sensor_fusion_status;
    sensor_fusion_status.status = status_;
    sensor_fusion_status.marker = false;
    pub_sensor_fusion_status_->publish(sensor_fusion_status);

    // Get Control Duration[s]
    double cnt_duration = controlFreqFluctuation();

    // Cancel previous timer
    timer_sensor_fusion_status_->cancel();

    // Timer for callback
    timer_sensor_fusion_status_ = this->create_wall_timer(
        std::chrono::duration<double>(cnt_duration),
        std::bind(&ib2::Nav::sensorFusionStatusCallBack, this), timer_group_);
}

//------------------------------------------------------------------------------
// 最新のオドメトリメッセージのコールバック関数
void ib2::Nav::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    latest_odom_ = msg;
}

//------------------------------------------------------------------------------
// 最新のIMUメッセージのコールバック関数
void ib2::Nav::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    latest_imu_ = msg;
}

//------------------------------------------------------------------------------
// 航法ステータスのサブスクライバのコールバック関数
void ib2::Nav::statusCallback(const example_interfaces::msg::Int32::SharedPtr status)
{
    if (status->data >= 0)
    {
        status_ = static_cast<uint8_t>(status->data);
        invalid_nav_ = false;
    }
    else
    {
        invalid_nav_ = true;
        status_ = static_cast<uint8_t>(-status->data);
    }
}

//------------------------------------------------------------------------------
// 航法時刻オフセットのサブスクライバのコールバック関数
void ib2::Nav::offsetCallback(const example_interfaces::msg::Float64::SharedPtr offset)
{
    tnav_offset_ = rclcpp::Duration::from_seconds(offset->data);
}

//------------------------------------------------------------------------------
// ロボットの航法値真値を取得する
ib2_interfaces::msg::Navigation ib2::Nav::getTrueNavigation()
{
    ib2_interfaces::msg::Navigation nav_msgs;
    nav_msgs.pose.header.stamp       = this->now() + tnav_offset_;
    nav_msgs.pose.header.frame_id    = FRAME_ISS;
    nav_msgs.status.status           = status_;

    if (latest_odom_ && latest_imu_) {
        // From odom: position, orientation, linear velocity
        nav_msgs.pose.pose.position      = latest_odom_->pose.pose.position;
        nav_msgs.pose.pose.orientation   = latest_odom_->pose.pose.orientation;
        nav_msgs.twist.linear            = latest_odom_->twist.twist.linear;
        nav_msgs.twist.angular           = latest_odom_->twist.twist.angular;

        // From imu: angular velocity, linear acceleration
        // nav_msgs.twist.angular           = latest_imu_->angular_velocity;
        nav_msgs.a                       = latest_imu_->linear_acceleration;
    } else {
        RCLCPP_WARN(this->get_logger(), "No odom or IMU data available");
        nav_msgs.pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();  // CHECK
    }

    return nav_msgs;
}

//------------------------------------------------------------------------------
// World(慣性)座標系からドッキングステーション(ホーム)座標系への座標変換
ib2_interfaces::msg::Navigation ib2::Nav::transformWtoH(const ib2_interfaces::msg::Navigation& nav_msgs)
{
    ib2_interfaces::msg::Navigation nav_msgs_h = nav_msgs;

    // If odom is already in docking station frame, this may be an identity transform
    try {
      geometry_msgs::msg::TransformStamped tf_iss = tf_buffer_.lookupTransform(
          nav_msgs.pose.header.frame_id, "docking_station", this->now());
      tf2::Transform iss_tf, robot_tf, ds_tf;
      tf2::fromMsg(tf_iss.transform, iss_tf);
      tf2::fromMsg(nav_msgs.pose.pose, robot_tf);
      ds_tf = iss_tf.inverse() * robot_tf;

      geometry_msgs::msg::Pose ds_pose;
      tf2::toMsg(ds_tf, ds_pose);
      nav_msgs_h.pose.pose = ds_pose;

      // Transform linear velocity
      geometry_msgs::msg::Vector3 world_vel = nav_msgs.twist.linear;
      tf2::Vector3 vel_tf(world_vel.x, world_vel.y, world_vel.z);
      vel_tf = iss_tf.inverse().getBasis() * vel_tf;
      nav_msgs_h.twist.linear.x = vel_tf.x();
      nav_msgs_h.twist.linear.y = vel_tf.y();
      nav_msgs_h.twist.linear.z = vel_tf.z();

      // Angular velocity and acceleration remain in body frame
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
    }

    return nav_msgs_h;
}

//------------------------------------------------------------------------------
// ROS Parameter Serverからパラメータを取得
int ib2::Nav::getParameter()
{
    int ret = 0;
    double x1, y1, z1;
    double x2, y2, z2;

    // Navigation Error Parameter
    this->declare_parameter("nav_parameter.add_error", true);
    add_error_ = this->get_parameter("nav_parameter.add_error").as_bool();

    this->declare_parameter("nav_parameter.error.error_source_csv", false);
    error_source_csv_ = this->get_parameter("nav_parameter.error.error_source_csv").as_bool();
    // if(!error_source_csv_)
    // {
    //     RCLCPP_WARN(this->get_logger(), "Could not read the parameters of \"nav_parameter.error.error_source_csv\".");
    //     ret   = ret | 0x0011;
    // }

    this->declare_parameter("nav_parameter.error.csv", "config/nav_error.csv");
    csv_file_name_ = this->get_parameter("nav_parameter.error.csv").as_string();
    if(csv_file_name_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Could not read the parameters of \"nav_parameter.error.csv\".");
        ret   = ret | 0x0012;
    }

    this->declare_parameter("nav_parameter.error.pos.mean.x", 0.0);
    this->declare_parameter("nav_parameter.error.pos.mean.y", 0.0);
    this->declare_parameter("nav_parameter.error.pos.mean.z", 0.0);
    this->declare_parameter("nav_parameter.error.pos.stddev.x", 0.0);
    this->declare_parameter("nav_parameter.error.pos.stddev.y", 0.0);
    this->declare_parameter("nav_parameter.error.pos.stddev.z", 0.0);
    x1 = this->get_parameter("nav_parameter.error.pos.mean.x").as_double();
    y1 = this->get_parameter("nav_parameter.error.pos.mean.y").as_double();
    z1 = this->get_parameter("nav_parameter.error.pos.mean.z").as_double();
    x2 = this->get_parameter("nav_parameter.error.pos.stddev.x").as_double();
    y2 = this->get_parameter("nav_parameter.error.pos.stddev.y").as_double();
    z2 = this->get_parameter("nav_parameter.error.pos.stddev.z").as_double();
    if( !x1 && !y1 && !z1 && !x2 && !y2 && !z2 )
    {
        RCLCPP_WARN(this->get_logger(), "Could not read the parameters of \"nav_parameter.error.pos\".");
        ret   = ret | 0x0014;
    }
    bias_p_.x = x1;
    bias_p_.y = y1;
    bias_p_.z = z1;
    rand_p_.x = x2;
    rand_p_.y = y2;
    rand_p_.z = z2;

    this->declare_parameter("nav_parameter.error.vel.mean.x", 0.0);
    this->declare_parameter("nav_parameter.error.vel.mean.y", 0.0);
    this->declare_parameter("nav_parameter.error.vel.mean.z", 0.0);
    this->declare_parameter("nav_parameter.error.vel.stddev.x", 0.0);
    this->declare_parameter("nav_parameter.error.vel.stddev.y", 0.0);
    this->declare_parameter("nav_parameter.error.vel.stddev.z", 0.0);
    x1 = this->get_parameter("nav_parameter.error.vel.mean.x").as_double();
    y1 = this->get_parameter("nav_parameter.error.vel.mean.y").as_double();
    z1 = this->get_parameter("nav_parameter.error.vel.mean.z").as_double();
    x2 = this->get_parameter("nav_parameter.error.vel.stddev.x").as_double();
    y2 = this->get_parameter("nav_parameter.error.vel.stddev.y").as_double();
    z2 = this->get_parameter("nav_parameter.error.vel.stddev.z").as_double();
    if( !x1 && !y1 && !z1 && !x2 && !y2 && !z2 )
    {
        RCLCPP_WARN(this->get_logger(), "Could not read the parameters of \"nav_parameter.error.vel\".");
        ret   = ret | 0x0018;
    }
    bias_v_.x = x1;
    bias_v_.y = y1;
    bias_v_.z = z1;
    rand_v_.x = x2;
    rand_v_.y = y2;
    rand_v_.z = z2;

    this->declare_parameter("nav_parameter.error.acc.mean.x", 0.0);
    this->declare_parameter("nav_parameter.error.acc.mean.y", 0.0);
    this->declare_parameter("nav_parameter.error.acc.mean.z", 0.0);
    this->declare_parameter("nav_parameter.error.acc.stddev.x", 0.0);
    this->declare_parameter("nav_parameter.error.acc.stddev.y", 0.0);
    this->declare_parameter("nav_parameter.error.acc.stddev.z", 0.0);
    x1 = this->get_parameter("nav_parameter.error.acc.stddev.z").as_double();
    y1 = this->get_parameter("nav_parameter.error.acc.stddev.y").as_double();
    z1 = this->get_parameter("nav_parameter.error.acc.stddev.x").as_double();
    x2 = this->get_parameter("nav_parameter.error.acc.mean.x").as_double();
    y2 = this->get_parameter("nav_parameter.error.acc.mean.y").as_double();
    z2 = this->get_parameter("nav_parameter.error.acc.mean.z").as_double();
    if( !x1 && !y1 && !z1 && !x2 && !y2 && !z2 )
    {
        RCLCPP_WARN(this->get_logger(), "Could not read the parameters of \"nav_parameter.error.acc\".");
        ret   = ret | 0x001F;
    }
    bias_a_.x = x1;
    bias_a_.y = y1;
    bias_a_.z = z1;
    rand_a_.x = x2;
    rand_a_.y = y2;
    rand_a_.z = z2;

    this->declare_parameter("nav_parameter.error.att.mean.x", 0.0);
    this->declare_parameter("nav_parameter.error.att.mean.y", 0.0);
    this->declare_parameter("nav_parameter.error.att.mean.z", 0.0);
    this->declare_parameter("nav_parameter.error.att.stddev.x", 0.0);
    this->declare_parameter("nav_parameter.error.att.stddev.y", 0.0);
    this->declare_parameter("nav_parameter.error.att.stddev.z", 0.0);
    x1 = this->get_parameter("nav_parameter.error.att.stddev.x").as_double();
    y1 = this->get_parameter("nav_parameter.error.att.stddev.y").as_double();
    z1 = this->get_parameter("nav_parameter.error.att.stddev.z").as_double();
    x2 = this->get_parameter("nav_parameter.error.att.mean.x").as_double();
    y2 = this->get_parameter("nav_parameter.error.att.mean.y").as_double();
    z2 = this->get_parameter("nav_parameter.error.att.mean.z").as_double();
    if( !x1 && !y1 && !z1 && !x2 && !y2 && !z2 )
    {
        RCLCPP_WARN(this->get_logger(), "Could not read the parameters of \"nav_parameter.error.att\".");
        ret   = ret | 0x0020;
    }
    bias_r_.x = x1;
    bias_r_.y = y1;
    bias_r_.z = z1;
    rand_r_.x = x2;
    rand_r_.y = y2;
    rand_r_.z = z2;


    this->declare_parameter("nav_parameter.error.att_rate.mean.x", 0.0);
    this->declare_parameter("nav_parameter.error.att_rate.mean.y", 0.0);
    this->declare_parameter("nav_parameter.error.att_rate.mean.z", 0.0);
    this->declare_parameter("nav_parameter.error.att_rate.stddev.x", 0.0);
    this->declare_parameter("nav_parameter.error.att_rate.stddev.y", 0.0);
    this->declare_parameter("nav_parameter.error.att_rate.stddev.z", 0.0);
    x1 = this->get_parameter("nav_parameter.error.att_rate.stddev.x").as_double();
    y1 = this->get_parameter("nav_parameter.error.att_rate.stddev.y").as_double();
    z1 = this->get_parameter("nav_parameter.error.att_rate.stddev.z").as_double();
    x2 = this->get_parameter("nav_parameter.error.att_rate.mean.x").as_double();
    y2 = this->get_parameter("nav_parameter.error.att_rate.mean.y").as_double();
    z2 = this->get_parameter("nav_parameter.error.att_rate.mean.z").as_double();
    if( x1 && !y1 && !z1 && !x2 && !y2 && !z2 )
    {
        RCLCPP_WARN(this->get_logger(), "Could not read the parameters of \"nav_parameter.error.att_rate\".");
        ret   = ret | 0x0021;
    }
    bias_w_.x = x1;
    bias_w_.y = y1;
    bias_w_.z = z1;
    rand_w_.x = x2;
    rand_w_.y = y2;
    rand_w_.z = z2;

    // Control Frequency Fluctuation Parameter
    this->declare_parameter("nav_parameter.control.mean", 0.0);
    this->declare_parameter("nav_parameter.control.stddev", 0.0);
    this->declare_parameter("nav_parameter.control.gain", 0.0);
    this->declare_parameter("nav_parameter.control.freq", 0.0);
    bias_cnt_ = this->get_parameter("nav_parameter.control.mean").as_double();
    rand_cnt_ = this->get_parameter("nav_parameter.control.stddev").as_double();
    gain_cnt_ = this->get_parameter("nav_parameter.control.gain").as_double();
    freq_cnt_ = this->get_parameter("nav_parameter.control.freq").as_double();
    if( !bias_cnt_ && !rand_cnt_ && !gain_cnt_ && !freq_cnt_ )
    {
        RCLCPP_WARN(this->get_logger(), "Could not read the parameters of \"nav_parameter.control\".");
        ret   = ret | 0x0022;
    }

    // Navigation Delay Parameter
    this->declare_parameter("nav_parameter.delay", -1.0);
    delay_ = this->get_parameter("nav_parameter.delay").as_double();
    if( delay_ < 0.0 )
    {
        RCLCPP_WARN(this->get_logger(), "Could not read the parameters of \"nav_parameter.delay\".");
        ret   = ret | 0x0024;
    }

    // Initial state of Navigation (true=ON)
    this->declare_parameter("nav_parameter.initial_nav_on", true);
    initial_nav_on_ = this->get_parameter("nav_parameter.initial_nav_on").as_bool();
    // if( !initial_nav_on_ )
    // {
    //     RCLCPP_WARN(this->get_logger(), "Could not read the parameters of \"nav_parameter.initial_nav_on\".");
    //     ret   = ret | 0x0028;
    // }

    // TODO: 乱数のシード値が設定されている場合は読み込む
    // int seed = -1;

    // マーカーID
    this->declare_parameter("nav_parameter.marker", 0);
    marker_ = this->get_parameter("nav_parameter.marker").as_int();

    return ret;
}

//------------------------------------------------------------------------------
// ROS Parameter Serverからパラメータを取得
bool ib2::Nav::updateParameter(
    const std::shared_ptr<ib2_interfaces::srv::UpdateParameter::Request> req,
    std::shared_ptr<ib2_interfaces::srv::UpdateParameter::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "Updating parameters");

    if (getParameter())
    {
        RCLCPP_INFO(this->get_logger(), "Successfully updated parameters");
        res->status = ib2_interfaces::srv::UpdateParameter::Response::SUCCESS;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to update parameters");
        res->status = ib2_interfaces::srv::UpdateParameter::Response::FAILURE_UPDATE;
    }

    if(error_source_csv_)
    {
        openCSVFile();
    }

    res->stamp = this->now();

    return true;
}

//------------------------------------------------------------------------------
// ROS Parameter Serverからパラメータを取得
bool ib2::Nav::markerCorrection(
    const std::shared_ptr<ib2_interfaces::srv::MarkerCorrection::Request> req,
    std::shared_ptr<ib2_interfaces::srv::MarkerCorrection::Response> res)
{
    res->stamp = this->now(); // CHECK: req might be updated, but srv is blank
    res->status = marker_ > 0 ?
        ib2_interfaces::srv::MarkerCorrection::Response::SUCCESS :
        ib2_interfaces::srv::MarkerCorrection::Response::FAILURE_UPDATE;
    return true;
}

//------------------------------------------------------------------------------
// Nav ON/OFF ServerからON/OFFコマンドを取得
bool ib2::Nav::switchPower(
        const std::shared_ptr<ib2_interfaces::srv::SwitchPower::Request> req,
        std::shared_ptr<ib2_interfaces::srv::SwitchPower::Response> res)
{
    if((status_ == ib2_interfaces::msg::NavigationStatus::NAV_OFF    && req->power.status == ib2_interfaces::msg::PowerStatus::OFF) || 
       (status_ == ib2_interfaces::msg::NavigationStatus::NAV_FUSION && req->power.status == ib2_interfaces::msg::PowerStatus::ON))
    {
        // ステータス変更無し
        res->current_power.status = req->power.status;
        return true;
    }

    if(req->power.status == ib2_interfaces::msg::PowerStatus::OFF)
    {
        timer_->cancel();
        status_                   = ib2_interfaces::msg::NavigationStatus::NAV_OFF;
        res->current_power.status = ib2_interfaces::msg::PowerStatus::OFF;

        while(!nav_buffer_.empty())
        {
            nav_buffer_.pop();
        }

        return true;
    }

    timer_->reset();

    if(status_ == ib2_interfaces::msg::NavigationStatus::NAV_OFF)
    {
        status_                   = ib2_interfaces::msg::NavigationStatus::NAV_FUSION;
        res->current_power.status = ib2_interfaces::msg::PowerStatus::ON;
    }

    return true;
}

// 航法機能アクション（実機固有インタフェース）模擬
void ib2::Nav::navigationStartUpCallback(
    const std::shared_ptr<GoalHandleNavigationStartUp> goal_handle)
{
    auto goal = goal_handle->get_goal();
    ib2_interfaces::action::NavigationStartUp::Result::SharedPtr result = std::make_shared<ib2_interfaces::action::NavigationStartUp::Result>();

    if(goal->command == ib2_interfaces::action::NavigationStartUp::Goal::ON)
    {
        // SwitchPower: ON
        ib2_interfaces::srv::SwitchPower::Request::SharedPtr switch_power_request = std::make_shared<ib2_interfaces::srv::SwitchPower::Request>();
        switch_power_request->power.status = ib2_interfaces::msg::PowerStatus::ON;
        ib2_interfaces::srv::SwitchPower::Response::SharedPtr switch_power_response = std::make_shared<ib2_interfaces::srv::SwitchPower::Response>();
        switchPower(switch_power_request, switch_power_response);

        // auto sleep_second = ignition::math::Rand::DblUniform(1.0, 2.5);
        std::uniform_real_distribution<double> dist(1.0, 2.5);
        double sleep_second = dist(rand_gen_);
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(sleep_second)));

        // Response result
        result->stamp = this->now();
        result->type = ib2_interfaces::action::NavigationStartUp::Result::ON_READY;
        goal_handle->succeed(result);
    }
    else if(goal->command == ib2_interfaces::action::NavigationStartUp::Goal::OFF)
    {
        // SwitchPower: OFF
        ib2_interfaces::srv::SwitchPower::Request::SharedPtr switch_power_request = std::make_shared<ib2_interfaces::srv::SwitchPower::Request>();
        switch_power_request->power.status = ib2_interfaces::msg::PowerStatus::OFF;
        ib2_interfaces::srv::SwitchPower::Response::SharedPtr switch_power_response = std::make_shared<ib2_interfaces::srv::SwitchPower::Response>();
        switchPower(switch_power_request, switch_power_response);

        // auto sleep_second = ignition::math::Rand::DblUniform(0.5, 1.5);
        std::uniform_real_distribution<double> dist(0.5, 1.5);
        double sleep_second = dist(rand_gen_);
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(sleep_second)));

        // Response result
        result->stamp = this->now();
        result->type = ib2_interfaces::action::NavigationStartUp::Result::OFF;
        goal_handle->succeed(result);
    }
    else
    {
        // Response result (ABORTED)
        result->stamp = this->now();
        result->type = ib2_interfaces::action::NavigationStartUp::Result::ABORTED;
        goal_handle->succeed(result);  // CHECK
        // goal_handle->abort(result);
    }
}

//------------------------------------------------------------------------------
// 航法値に誤差を付加する
ib2_interfaces::msg::Navigation ib2::Nav::addError(const ib2_interfaces::msg::Navigation& nav_msgs)
{
    ib2_interfaces::msg::Navigation nav_msgs_e = nav_msgs;
    ib2::Nav::NavError              nav_error  = getNavError();

    // Add noise
    nav_msgs_e.pose.pose.position.x += nav_error.pos.x;
    nav_msgs_e.pose.pose.position.y += nav_error.pos.y;
    nav_msgs_e.pose.pose.position.z += nav_error.pos.z;
    nav_msgs_e.twist.linear.x       += nav_error.vel.x;
    nav_msgs_e.twist.linear.y       += nav_error.vel.y;
    nav_msgs_e.twist.linear.z       += nav_error.vel.z;
    nav_msgs_e.a.x                  += nav_error.acc.x;
    nav_msgs_e.a.y                  += nav_error.acc.y;
    nav_msgs_e.a.z                  += nav_error.acc.z;

// Add orientation error
    tf2::Quaternion q, q_err;
    tf2::fromMsg(nav_msgs_e.pose.pose.orientation, q);
    q_err.setRPY(nav_error.rot.x * DEG2RAD, nav_error.rot.y * DEG2RAD, nav_error.rot.z * DEG2RAD);
    q = q * q_err;
    nav_msgs_e.pose.pose.orientation = tf2::toMsg(q);

    nav_msgs_e.twist.angular.x      += nav_error.w.x * DEG2RAD;
    nav_msgs_e.twist.angular.y      += nav_error.w.y * DEG2RAD;
    nav_msgs_e.twist.angular.z      += nav_error.w.z * DEG2RAD;

    return nav_msgs_e;
}

//------------------------------------------------------------------------------
// 制御周期の変動を模擬する
double ib2::Nav::controlFreqFluctuation()
{
    // Fluctuated Control Frequency[Hz]
    rclcpp::Time t    = this->now();
    std::normal_distribution<double> dist(bias_cnt_, rand_cnt_);
    double       wg   = dist(rand_gen_);
    double       freq = gain_cnt_ * sin(2.0 * M_PI * freq_cnt_ * t.seconds()) + wg;

    // Convert Freq[Hz] to Duration[s]
    assert(std::abs(freq) > EPS);
    double duration = 1.0 / freq;

    return duration;
}

//------------------------------------------------------------------------------
// NavigationメッセージからAttitudeメッセージを作成する
ib2_interfaces::msg::Attitude ib2::Nav::makeAttMsgFromNavMsg(const ib2_interfaces::msg::Navigation& nav_msgs)
{
    // Make Attitude Message
    ib2_interfaces::msg::Attitude att_msgs;
    att_msgs.stamp = nav_msgs.pose.header.stamp;
    att_msgs.q     = nav_msgs.pose.pose.orientation;
    att_msgs.w.x   = nav_msgs.twist.angular.x * RAD2DEG;
    att_msgs.w.y   = nav_msgs.twist.angular.y * RAD2DEG;
    att_msgs.w.z   = nav_msgs.twist.angular.z * RAD2DEG;

    tf2::Quaternion q;
    tf2::fromMsg(nav_msgs.pose.pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    att_msgs.euler.x = roll  * RAD2DEG;
    att_msgs.euler.y = pitch * RAD2DEG;
    att_msgs.euler.z = yaw   * RAD2DEG;

    return att_msgs;
}

//------------------------------------------------------------------------------
// 航法誤差を取得する
ib2::Nav::NavError ib2::Nav::getNavError()
{
    ib2::Nav::NavError nav_error;

    // From CSV
    if(error_source_csv_ && ifs_.is_open())
    {
        std::string line;
        std::getline(ifs_, line);

        if(ifs_.eof())
        {
            RCLCPP_WARN(this->get_logger(), "End of CSV file, resetting to top");
            ifs_.clear();
            ifs_.seekg(0, std::ios_base::beg);
            std::getline(ifs_, line);
        }

        // skip comment line
        if(line.find_first_of('#') == 0)
        {
            std::getline(ifs_, line);  // Read unit line
            std::getline(ifs_, line);  // Read error value
        }

        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        double val[15];
        for(int i = 0; i < 15; i++)
        {
            iss >> val[i];
        }
        nav_error.pos.x = val[0];
        nav_error.pos.y = val[1];
        nav_error.pos.z = val[2];
        nav_error.vel.x = val[3];
        nav_error.vel.y = val[4];
        nav_error.vel.z = val[5];
        nav_error.acc.x = val[6];
        nav_error.acc.y = val[7];
        nav_error.acc.z = val[8];
        nav_error.rot.x = val[9];
        nav_error.rot.y = val[10];
        nav_error.rot.z = val[11];
        nav_error.w.x   = val[12];
        nav_error.w.y   = val[13];
        nav_error.w.z   = val[14];
    }

    // From Random Number
    else
    {
        std::normal_distribution<double> dist(0.0, 1.0);
        nav_error.pos.x = dist(rand_gen_) * rand_p_.x + bias_p_.x;
        nav_error.pos.y = dist(rand_gen_) * rand_p_.y + bias_p_.y;
        nav_error.pos.z = dist(rand_gen_) * rand_p_.z + bias_p_.z;
        nav_error.vel.x = dist(rand_gen_) * rand_v_.x + bias_v_.x;
        nav_error.vel.y = dist(rand_gen_) * rand_v_.y + bias_v_.y;
        nav_error.vel.z = dist(rand_gen_) * rand_v_.z + bias_v_.z;
        nav_error.acc.x = dist(rand_gen_) * rand_a_.x + bias_a_.x;
        nav_error.acc.y = dist(rand_gen_) * rand_a_.y + bias_a_.y;
        nav_error.acc.z = dist(rand_gen_) * rand_a_.z + bias_a_.z;
        nav_error.rot.x = dist(rand_gen_) * rand_r_.x + bias_r_.x;
        nav_error.rot.y = dist(rand_gen_) * rand_r_.y + bias_r_.y;
        nav_error.rot.z = dist(rand_gen_) * rand_r_.z + bias_r_.z;
        nav_error.w.x   = dist(rand_gen_) * rand_w_.x + bias_w_.x;
        nav_error.w.y   = dist(rand_gen_) * rand_w_.y + bias_w_.y;
        nav_error.w.z   = dist(rand_gen_) * rand_w_.z + bias_w_.z;
    }

    return nav_error;
}

//------------------------------------------------------------------------------
// 航法誤差CSVファイルオープン
void ib2::Nav::openCSVFile()
{
    if(!error_source_csv_ || ifs_.is_open())
    {
        return;
    }
    // Get Plugin Path
    std::string plugin_path = ament_index_cpp::get_package_share_directory("ib2_nav") + "/";

    ifs_.open(plugin_path + csv_file_name_, std::ios::in);

    if(ifs_.fail() || csv_file_name_.empty())
    {
        error_source_csv_ = false;
        RCLCPP_WARN(this->get_logger(), "Navigation Error will be added by generating random number\n");
    }
}

// End Of File -----------------------------------------------------------------
