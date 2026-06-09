"""
NeoAssist Phase 2 — SLAM Mapping
SLAM Toolbox is a lifecycle node in Jazzy — must be activated.
We use the non-lifecycle version: sync_slam_toolbox_node
OR we activate it via lifecycle manager.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg       = get_package_share_directory("neo_assist")
    urdf_file = os.path.join(pkg, "urdf",    "neo_robot.urdf.xacro")
    world_file= os.path.join(pkg, "worlds",   "hospital.sdf")
    slam_cfg  = os.path.join(pkg, "config",   "slam_config.yaml")
    rviz_file = os.path.join(pkg, "rviz",     "phase2_slam.rviz")

    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]),
        value_type=str
    )

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen"
    )

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
            ("/world/hospital/model/neo_robot/joint_state", "/joint_states"),
        ],
        output="screen"
    )

    odom_tf_broadcaster = Node(
        package="neo_assist",
        executable="odom_tf_broadcaster",
        name="odom_tf_broadcaster",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    scan_frame_fixer = Node(
        package="neo_assist",
        executable="scan_frame_fixer",
        name="scan_frame_fixer",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # Use LifecycleNode + lifecycle_manager to auto-activate SLAM
    slam_toolbox = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[slam_cfg],
        remappings=[("/scan", "/scan_fixed")],
    )

    # Lifecycle manager auto-configures and activates SLAM
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": ["slam_toolbox"],
        }]
    )

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

    rviz = TimerAction(
        period=8.0,
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
        slam_toolbox,
        lifecycle_manager,
        spawn_robot,
        rviz,
    ])
