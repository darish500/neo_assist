"""
NeoAssist Phase 1 Launch — Final stable version
=================================================
Key fixes:
  1. gz_frame_id in URDF fixes LiDAR frame at source
  2. scan_frame_fixer node relays /scan -> /scan_fixed with
     correct frame_id = lidar_link (belt and suspenders)
  3. RViz reads /scan_fixed (RELIABLE QoS, correct frame)
  4. odom_tf_broadcaster no longer crashes on shutdown
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg        = get_package_share_directory("neo_assist")
    urdf_file  = os.path.join(pkg, "urdf",   "neo_robot.urdf.xacro")
    world_file = os.path.join(pkg, "worlds",  "hospital.sdf")
    rviz_file  = os.path.join(pkg, "rviz",    "neo_assist.rviz")

    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]),
        value_type=str
    )

    # 1. Gazebo
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen"
    )

    # 2. Robot State Publisher — static URDF TF frames
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }]
    )

    # 3. ROS-GZ Bridge
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/world/hospital/model/neo_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        parameters=[{
            "use_sim_time": True,
            "qos_overrides./scan.publisher.reliability": "best_effort",
            "qos_overrides./scan.publisher.durability": "volatile",
        }],
        remappings=[
            # Remap Gazebo scoped joint topic -> standard /joint_states
            ("/world/hospital/model/neo_robot/joint_state", "/joint_states"),
        ],
        output="screen"
    )

    # 4. Odom TF broadcaster — odom -> base_footprint using sim time
    odom_tf_broadcaster = Node(
        package="neo_assist",
        executable="odom_tf_broadcaster",
        name="odom_tf_broadcaster",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # 5. Scan frame fixer — /scan (wrong frame) -> /scan_fixed (lidar_link)
    #    Also converts BEST_EFFORT -> RELIABLE for RViz and SLAM
    scan_frame_fixer = Node(
        package="neo_assist",
        executable="scan_frame_fixer",
        name="scan_frame_fixer",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # 6. Static TF map -> odom (placeholder until SLAM runs)
    static_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_odom",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "map",
                   "--child-frame-id", "odom"],
        parameters=[{"use_sim_time": True}]
    )

    # 7. Spawn robot after 4 s
    spawn_robot = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_neo_robot",
                arguments=[
                    "-name",  "neo_robot",
                    "-topic", "robot_description",
                    "-x",     "-8.0",
                    "-y",      "0.0",
                    "-z",      "0.50",
                    "-Y",      "0.0",
                ],
                output="screen"
            )
        ]
    )

    # 8. RViz after 3 s
    rviz = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_file],
                parameters=[{"use_sim_time": True}],
                output="screen"
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        ros_gz_bridge,
        odom_tf_broadcaster,
        scan_frame_fixer,
        static_map_odom,
        spawn_robot,
        rviz,
    ])
