#!/usr/bin/env python3

import argparse
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer as Writer2
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.msg import get_types_from_msg

def create_custom_typestore():
    """
    Creates a single, unified typestore containing all necessary message definitions.
    """
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    message_definitions = {
        # --- Old ib2_msgs ---
        'ib2_msgs/msg/CtlStatusType': "int32 STAND_BY=0\nint32 CAPTURED=1\nint32 DISTURBED=2\nint32 KEEP_POSE=10\nint32 KEEPING_POSE_BY_COLLISION=11\nint32 STOP_MOVING=20\nint32 MOVE_TO_RELATIVE_TARGET=30\nint32 MOVE_TO_ABSOLUTE_TARGET=40\nint32 RELEASE=50\nint32 DOCK=60\nint32 MOVING_TO_AIA_AIP=61\nint32 WAITING_AT_AIA_AIP=62\nint32 MOVING_TO_RDA_AIP=63\nint32 MOVING_TO_RDP=64\nint32 DOCKING_STAND_BY=65\nint32 DOCK_WITHOUT_CORRECTION=69\nint32 SCAN=70\nint32 type",
        'ib2_msgs/msg/CtlStatus': "geometry_msgs/PoseStamped pose\ngeometry_msgs/Twist twist\ngeometry_msgs/Vector3 a\nib2_msgs/CtlStatusType type",
        'ib2_msgs/msg/PowerStatus': "uint8 OFF=0\nuint8 ON=1\nuint8 UNKNOWN=2\nuint8 status",
        'ib2_msgs/msg/FanStatus': "std_msgs/Header header\nstd_msgs/Float64MultiArray duty\nib2_msgs/PowerStatus current_power",
        'ib2_msgs/msg/IMU': "time stamp\nfloat32 acc_x\nfloat32 acc_y\nfloat32 acc_z\nfloat32 gyro_x\nfloat32 gyro_y\nfloat32 gyro_z\nfloat32 temperature",
        'ib2_msgs/msg/Slam': "time stamp\nint32 slam_status\nfloat32 x_s\nfloat32 y_s\nfloat32 z_s\nfloat32 v_x\nfloat32 v_y\nfloat32 v_z\nfloat32 qx\nfloat32 qy\nfloat32 qz\nfloat32 qw\nfloat32 w_x\nfloat32 w_y\nfloat32 w_z\nuint32 point",
        'ib2_msgs/msg/NavigationStatus': "uint8 NAV_OFF=0\nuint8 NAV_FUSION=1\nuint8 NAV_INERTIAL=2\nuint8 NAV_SLAM=3\nuint8 status\nbool marker",
        'ib2_msgs/msg/Navigation': "geometry_msgs/PoseStamped pose\ngeometry_msgs/Twist twist\ngeometry_msgs/Vector3 a\nib2_msgs/NavigationStatus status",
        'ib2_msgs/msg/CtlProfile': "std_msgs/Header header\ngeometry_msgs/PoseStamped[] poses",

        # --- Standard ROS2 message dependencies ---
        'example_interfaces/msg/MultiArrayDimension': "string label\nuint32 size\nuint32 stride",
        'example_interfaces/msg/MultiArrayLayout': "example_interfaces/msg/MultiArrayDimension[] dim\nuint32 data_offset",
        'example_interfaces/msg/Float64MultiArray': "example_interfaces/msg/MultiArrayLayout layout\nfloat64[] data",

        # --- New ib2_interfaces ---
        'ib2_interfaces/msg/CtlStatusType': "int32 STAND_BY=0\nint32 CAPTURED=1\nint32 DISTURBED=2\nint32 KEEP_POSE=10\nint32 KEEPING_POSE_BY_COLLISION=11\nint32 STOP_MOVING=20\nint32 MOVE_TO_RELATIVE_TARGET=30\nint32 MOVE_TO_ABSOLUTE_TARGET=40\nint32 RELEASE=50\nint32 DOCK=60\nint32 MOVING_TO_AIA_AIP=61\nint32 WAITING_AT_AIA_AIP=62\nint32 MOVING_TO_RDA_AIP=63\nint32 MOVING_TO_RDP=64\nint32 DOCKING_STAND_BY=65\nint32 DOCK_WITHOUT_CORRECTION=69\nint32 SCAN=70\nint32 type",
        'ib2_interfaces/msg/CtlStatus': "geometry_msgs/PoseStamped pose\ngeometry_msgs/Twist twist\ngeometry_msgs/Vector3 a\nib2_interfaces/CtlStatusType type",
        'ib2_interfaces/msg/PowerStatus': "uint8 OFF=0\nuint8 ON=1\nuint8 UNKNOWN=2\nuint8 status",
        'ib2_interfaces/msg/FanStatus': "std_msgs/Header header\nexample_interfaces/Float64MultiArray duty\nib2_interfaces/PowerStatus current_power",
        'ib2_interfaces/msg/IMU': "builtin_interfaces/Time stamp\nfloat32 acc_x\nfloat32 acc_y\nfloat32 acc_z\nfloat32 gyro_x\nfloat32 gyro_y\nfloat32 gyro_z\nfloat32 temperature",
        'ib2_interfaces/msg/Slam': "builtin_interfaces/Time stamp\nint32 slam_status\nfloat32 x_s\nfloat32 y_s\nfloat32 z_s\nfloat32 v_x\nfloat32 v_y\nfloat32 v_z\nfloat32 qx\nfloat32 qy\nfloat32 qz\nfloat32 qw\nfloat32 w_x\nfloat32 w_y\nfloat32 w_z\nuint32 point",
        'ib2_interfaces/msg/NavigationStatus': "uint8 NAV_OFF=0\nuint8 NAV_FUSION=1\nuint8 NAV_INERTIAL=2\nuint8 NAV_SLAM=3\nuint8 status\nbool marker",
        'ib2_interfaces/msg/Navigation': "geometry_msgs/PoseStamped pose\ngeometry_msgs/Twist twist\ngeometry_msgs/Vector3 a\nib2_interfaces/NavigationStatus status",
        'ib2_interfaces/msg/CtlProfile': "std_msgs/Header header\ngeometry_msgs/PoseStamped[] poses",
    }
    
    for name, text in message_definitions.items():
        typestore.register(get_types_from_msg(text, name))
        
    return typestore

def main():
    # --- Argument Parsing ---
    parser = argparse.ArgumentParser(description='Convert a ROS1 bag with ib2_msgs to a ROS2 bag with ib2_interfaces.')
    parser.add_argument('input_bag', type=Path, help='Path to the input ROS1 bag file.')
    parser.add_argument('output_bag', type=Path, help='Path to the output ROS2 bag directory (will be created).')
    args = parser.parse_args()

    src_bag_path = args.input_bag
    dst_bag_path = args.output_bag
    # --- End of Argument Parsing ---

    if not src_bag_path.exists():
        print(f"Error: Source bag file not found at '{src_bag_path}'")
        return
        
    if dst_bag_path.exists():
        print(f"Error: Output directory '{dst_bag_path}' already exists. Please remove it or choose a different path.")
        return

    custom_typestore = create_custom_typestore()
    
    type_rename_map = {
        'ib2_msgs/msg/CtlStatus': 'ib2_interfaces/msg/CtlStatus',
        'ib2_msgs/msg/FanStatus': 'ib2_interfaces/msg/FanStatus',
        'ib2_msgs/msg/IMU': 'ib2_interfaces/msg/IMU',
        'ib2_msgs/msg/Slam': 'ib2_interfaces/msg/Slam',
        'ib2_msgs/msg/Navigation': 'ib2_interfaces/msg/Navigation',
        'ib2_msgs/msg/CtlProfile': 'ib2_interfaces/msg/CtlProfile',
    }
    
    # ADDED: List of suffixes for ROS1 and ROS2 action topics to be skipped
    action_suffixes_to_skip = [
        '/feedback', '/result', '/goal', '/cancel',
        '/command/status'
    ]
    
    print("Starting manual conversion...")
    
    with AnyReader([src_bag_path], default_typestore=custom_typestore) as reader, \
         Writer2(dst_bag_path, version=8) as writer:
        
        connection_map = {}
        for rconn in reader.connections:
            if any(rconn.topic.endswith(suffix) for suffix in action_suffixes_to_skip):
                print(f"  - Skipping action topic: '{rconn.topic}'")
                continue

            new_msgtype = type_rename_map.get(rconn.msgtype, rconn.msgtype)
            print(f"  - Mapping topic '{rconn.topic}' from [{rconn.msgtype}] to [{new_msgtype}]")
            connection_map[rconn.id] = writer.add_connection(
                topic=rconn.topic,
                msgtype=new_msgtype,
                typestore=custom_typestore
            )

        for rconn, timestamp, rawdata in reader.messages():
            if rconn.id not in connection_map:
                continue

            wconn = connection_map[rconn.id]

            if rconn.msgtype not in type_rename_map:
                writer.write(wconn, timestamp, rawdata)
                continue

            # --- CORRECTED TRANSFORMATION LOGIC ---
            # 1. Deserialize the old message
            msg = reader.deserialize(rawdata, rconn.msgtype)
            
            # 2. Get the Python class for the NEW message type
            NewMsgType = custom_typestore.types[wconn.msgtype]
            
            # 3. Create a list of arguments for the constructor
            #    by getting each required field's value from the old message.
            #    The field names and order are the same, so this works directly.
            constructor_args = [
                getattr(msg, field_name) 
                for field_name, _ in custom_typestore.fielddefs[wconn.msgtype][1]
            ]
            
            # 4. Create the new message object by passing the arguments to the constructor.
            #    The `*` unpacks the list into positional arguments.
            new_msg = NewMsgType(*constructor_args)
            
            # 5. Serialize the new message and write it to the bag.
            serialized_data = custom_typestore.serialize_cdr(new_msg, wconn.msgtype)
            writer.write(wconn, timestamp, serialized_data)
    
    print(f"\nConversion complete. Output saved to '{dst_bag_path}'")

if __name__ == '__main__':
    main()
