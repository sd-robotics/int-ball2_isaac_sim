
#pragma once

// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <gazebo/common/Plugin.hh>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <example_interfaces/msg/int32.hpp>
#include <example_interfaces/msg/float64.hpp>

#include "ib2_interfaces/msg/attitude.hpp"
#include "ib2_interfaces/msg/navigation.hpp"
#include "ib2_interfaces/msg/navigation_status.hpp"
#include "ib2_interfaces/action/navigation_start_up.hpp"
#include "ib2_interfaces/srv/marker_correction.hpp"
#include "ib2_interfaces/srv/switch_power.hpp"
#include "ib2_interfaces/srv/update_parameter.hpp"

#include <chrono>
#include <fstream>
#include <random>
// #include <cassert>
#include <queue>

namespace ib2
{
/**
 * @brief Robotの状態量を取得し、航法値メッセージをpublishするプラグイン.
 */
class Nav : public rclcpp::Node
{
    //----------------------------------------------------------------------
    // コンストラクタ/デストラクタ
public:

    using NavigationStartUp = ib2_interfaces::action::NavigationStartUp;
    using GoalHandleNavigationStartUp = rclcpp_action::ServerGoalHandle<NavigationStartUp>;

    /** デフォルトコンストラクタ */
    Nav(const rclcpp::NodeOptions& options);

    /** デストラクタ. */
    ~Nav();

    //----------------------------------------------------------------------
    // コピー/ムーブ
private:
    /** コピーコンストラクタ. */
    Nav(const Nav&)            = delete;

    /** コピー代入演算子. */
    Nav& operator=(const Nav&) = delete;

    /** ムーブコンストラクタ. */
    Nav(Nav&&)                 = delete;

    /** ムーブ代入演算子. */
    Nav& operator=(Nav&&)      = delete;

    //----------------------------------------------------------------------
    // 実装
public:
    /** プラグインのロード
     * @param [in, out] _world Worldへのポインタ
     * @param [in, out] _sdf SDF要素へのポインタ
     */
    // virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

private:
    /** ROS Parameter Serverからパラメータを取得 */
    int getParameter();

    /** Goalの受理
     * @param [in] uuid GoalのUUID
     * @param [in] goal 受理するGoal
     * @return rclcpp_action::GoalResponse
     */
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigationStartUp::Goal> goal);

    /** Goalのキャンセル
     * @param [in] goal_handle キャンセルするGoalのハンドル
     * @return rclcpp_action::CancelResponse
     */
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleNavigationStartUp> goal_handle);

    /** Goalの受理
     * @param [in] goal_handle 受理するGoalのハンドル
     */
    void handleAccepted(
        const std::shared_ptr<GoalHandleNavigationStartUp> goal_handle);

    /** 航法プラグインパラメータ更新
    * @param [in]                 req      パラメータ更新サービスリクエスト
    * @param [in]                 res      パラメータ更新サービス実行結果
    * @retval                     true     更新成功
    * @retval                     false    更新失敗
    */
    bool updateParameter(
        const std::shared_ptr<ib2_interfaces::srv::UpdateParameter::Request> req,
        std::shared_ptr<ib2_interfaces::srv::UpdateParameter::Response> res
    );

    /** マーカー補正
     * @param [in]                 req      マーカー補正サービスリクエスト
     * @param [in]                 res      マーカー補正サービス実行結果
     * @retval                     true     更新成功
     * @retval                     false    更新失敗
     */
    bool markerCorrection(
        const std::shared_ptr<ib2_interfaces::srv::MarkerCorrection::Request> req,
        std::shared_ptr<ib2_interfaces::srv::MarkerCorrection::Response> res
    );

    /** Nav ON/OFF
    * @param [in]                 req              Nav ON/OFFサービスリクエスト
    * @param [in]                 res              現在のNav ON/OFFステータス
    * @retval                     true             設定成功
    * @retval                     false            設定失敗
    */
    bool switchPower(
        const std::shared_ptr<ib2_interfaces::srv::SwitchPower::Request> req,
        std::shared_ptr<ib2_interfaces::srv::SwitchPower::Response> res
    );
    
    /** 航法誤差CSVファイルオープン */
    void openCSVFile();

    /** ISS/IB2モデルを取得 */
    // void getModels();

    /** ROS Timerのコールバック関数 */
    void navCallBack();

    /** ROS Timer（航法機能ステータス値（実機固有インタフェース）出力用）のコールバック関数 */
    void sensorFusionStatusCallBack();

    /** 最新のオドメトリメッセージのコールバック関数 */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /** 最新のIMUメッセージのコールバック関数 */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    /** 航法ステータスのサブスクライバのコールバック関数
     * @param [in] status 航法ステータス
     */
    void statusCallback(const example_interfaces::msg::Int32::SharedPtr status);

    /** 航法ステータスのサブスクライバのコールバック関数
     * @param [in] status 航法ステータス
     */
    void offsetCallback(const example_interfaces::msg::Float64::SharedPtr offset);

    /** ロボットの航法値真値を取得する
     * @return ib2_interfaces::msg::Navigation 航法値メッセージ
     */
    ib2_interfaces::msg::Navigation getTrueNavigation();

    /** World(慣性)座標系からホーム座標系への座標変換
     * @param [in, out] nav_msgs 航法値メッセージへの参照
     * @return ib2_interfaces::msg::Navigation 航法値メッセージ
     */
    ib2_interfaces::msg::Navigation transformWtoH(const ib2_interfaces::msg::Navigation& nav_msgs);

    /** 航法値に誤差を付加する
     * @param [in, out] nav_msgs 航法値メッセージへの参照
     * @return ib2_interfaces::msg::Navigation 誤差込み航法値メッセージ
     */
    ib2_interfaces::msg::Navigation addError(const ib2_interfaces::msg::Navigation& nav_msgs);

    /** 制御周期の変動を模擬する
     * @return double 変動込み制御周期[s]
     */
    double controlFreqFluctuation();

    /** NavigationメッセージからAttitudeメッセージを作成する
     * @param [in] nav_msgs       航法値メッセージへの参照
     * @return ib2_interfaces::msg::Attitude 姿勢情報メッセージ
     */
    ib2_interfaces::msg::Attitude makeAttMsgFromNavMsg(const ib2_interfaces::msg::Navigation& nav_msgs);

    /** 加速度、角加速度の加算（Gazeboのコールバック関数） */
    // void sumAcclAndAttRate();

    /** 航法誤差 */
    struct NavError
    {
        geometry_msgs::msg::Vector3 pos;
        geometry_msgs::msg::Vector3 vel;
        geometry_msgs::msg::Vector3 acc;
        geometry_msgs::msg::Vector3 rot;
        geometry_msgs::msg::Vector3 w;

        NavError()
        {
            pos = geometry_msgs::msg::Vector3();
            vel = geometry_msgs::msg::Vector3();
            acc = geometry_msgs::msg::Vector3();
            rot = geometry_msgs::msg::Vector3();
            w   = geometry_msgs::msg::Vector3();
        }
    };

    /** 航法誤差を取得する
     * @return NavError      航法誤差構造体
     */
    NavError getNavError();

    /** 航法機能アクション（実機固有インタフェース）模擬　受信時の処理
     * @param [in] request 受信メッセージ
     */
    void navigationStartUpCallback(const std::shared_ptr<GoalHandleNavigationStartUp> goal);

    //----------------------------------------------------------------------
    // メンバ変数
private:
    /* 航法誤差CSVファイルストリーム */
    std::ifstream                   ifs_;

    /** TFバッファ */
    tf2_ros::Buffer                 tf_buffer_;

    /** TFリスナー */
    tf2_ros::TransformListener      tf_listener_;

    /** 航法値(誤差込)のパブリッシャ */
    rclcpp::Publisher<ib2_interfaces::msg::Navigation>::SharedPtr       pub_nav_;

    /** 航法機能ステータスのパブリッシャ（実機固有インタフェースの模擬） */
    rclcpp::Publisher<ib2_interfaces::msg::NavigationStatus>::SharedPtr pub_sensor_fusion_status_;

    /** 姿勢状態量(誤差込)のパブリッシャ */
    rclcpp::Publisher<ib2_interfaces::msg::Attitude>::SharedPtr         pub_att_;

    /** 航法値真値のパブリッシャ */
    rclcpp::Publisher<ib2_interfaces::msg::Navigation>::SharedPtr       pub_true_nav_;

    /** 姿勢状態量真値のパブリッシャ */
    rclcpp::Publisher<ib2_interfaces::msg::Attitude>::SharedPtr         pub_true_att_;

    /** 航法オドメトリのサブスクライバ */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            sub_odom_;

    /** 航法IMUのサブスクライバ */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr              sub_imu_;

    /** 航法ステータスのサブスクライバ */
    rclcpp::Subscription<example_interfaces::msg::Int32>::SharedPtr     sub_status_;

    /** 航法時刻オフセットのサブスクライバ */
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr   sub_time_offset_;

    /** 航法プラグインパラメータ更新サービスサーバ */
    rclcpp::Service<ib2_interfaces::srv::UpdateParameter>::SharedPtr    nav_param_server_;

    /** マーカー補正サービスサーバ */
    rclcpp::Service<ib2_interfaces::srv::MarkerCorrection>::SharedPtr   marker_correction_server_;

    /** Nav ON/OFFサービスサーバ */
    rclcpp::Service<ib2_interfaces::srv::SwitchPower>::SharedPtr        switch_power_server_;

    /** タイマコールバックグループ */
    rclcpp::CallbackGroup::SharedPtr timer_group_;

    /** ROSタイマ */
    rclcpp::TimerBase::SharedPtr     timer_;

    /** ROSタイマ 航法機能ステータス値（実機固有インタフェース）出力用 */
    rclcpp::TimerBase::SharedPtr     timer_sensor_fusion_status_;

    /** 航法機能アクション（実機固有インタフェース）スタブ */
    rclcpp_action::Server<NavigationStartUp>::SharedPtr navigation_start_up_;

    /** 航法メッセージバッファ */
    std::queue<ib2_interfaces::msg::Navigation> nav_buffer_;

    /** 最新のオドメトリメッセージ */
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;

    /** 最新のIMUメッセージ */
    sensor_msgs::msg::Imu::SharedPtr   latest_imu_;

    /** 航法遅れ時間 */
    double                         delay_;

    /** 航法誤差追加フラグ */
    bool                           add_error_;

    /** 航法誤差源 */
    bool                           error_source_csv_;

    /**  位置バイアス誤差[m]*/
    geometry_msgs::msg::Vector3    bias_p_;

    /**  位置ランダム誤差[m]*/
    geometry_msgs::msg::Vector3    rand_p_;

    /**  速度バイアス誤差[m/s]*/
    geometry_msgs::msg::Vector3    bias_v_;

    /**  速度ランダム誤差[m/s]*/
    geometry_msgs::msg::Vector3    rand_v_;

    /**  加速度バイアス誤差[m/s^2]*/
    geometry_msgs::msg::Vector3    bias_a_;

    /**  加速度ランダム誤差[m/s^2]*/
    geometry_msgs::msg::Vector3    rand_a_;

    /**  姿勢バイアス誤差[deg]*/
    geometry_msgs::msg::Vector3    bias_r_;

    /**  姿勢ランダム誤差[deg]*/
    geometry_msgs::msg::Vector3    rand_r_;

    /**  角速度バイアス誤差[deg/s]*/
    geometry_msgs::msg::Vector3    bias_w_;

    /**  角速度ランダム誤差[deg/s]*/
    geometry_msgs::msg::Vector3    rand_w_;

    /** 前回位置 */
    geometry_msgs::msg::Point      prev_pos_;

    /**  ノミナル制御周期[Hz]*/
    double                         bias_cnt_;

    /**  制御周期ランダム誤差[Hz]*/
    double                         rand_cnt_;

    /**  制御周期変動ゲイン[-]*/
    double                         gain_cnt_;

    /**  制御周期変動周期[Hz]*/
    double                         freq_cnt_;

    /** 航法誤差CSVファイル名 */
    std::string                    csv_file_name_;

    /** 航法ステータス */
    uint8_t                        status_;

    /** マーカーID */
    uint8_t                        marker_;
    
    /** 航法時刻オフセット */
    rclcpp::Duration               tnav_offset_;

    /** 前回時刻 */
    rclcpp::Time                   prev_time_;

    /** 前回時刻の有無 */
    bool                           has_prev_;

    /** 航法データ異常値 */
    bool                           invalid_nav_;

    /** 航法出力の初期状態 **/
    bool                           initial_nav_on_;

    /** 乱数生成器 */
    std::mt19937                   rand_gen_;
};

} // namespace ib2

rclcpp_action::GoalResponse ib2::Nav::handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigationStartUp::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with command: %s",
                goal->command ? "ON(1)" : "OFF(0)");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ib2::Nav::handleCancel(
    const std::shared_ptr<ib2::Nav::GoalHandleNavigationStartUp> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ib2::Nav::handleAccepted(const std::shared_ptr<ib2::Nav::GoalHandleNavigationStartUp> goal_handle)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ib2::Nav::navigationStartUpCallback, this, std::placeholders::_1), goal_handle}.detach();
}


// Register the node with the rclcpp components system
RCLCPP_COMPONENTS_REGISTER_NODE(ib2::Nav)

// End Of File -----------------------------------------------------------------
