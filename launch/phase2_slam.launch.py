"""
NeoAssist Phase 2 — SLAM + Nav2 + Auto Explorer
================================================
Copied from warehouse_wall_follower pattern:
- SLAM via online_async_launch.py (no lifecycle issues)
- Nav2 WITHOUT amcl/map_server (SLAM provides map TF)
- Auto explorer navigates hospital autonomously
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg       = get_package_share_directory("neo_assist")
    slam_pkg  = get_package_share_directory("slam_toolbox")
    urdf_file = os.path.join(pkg, "urdf",   "neo_robot.urdf.xacro")
    world_file= os.path.join(pkg, "worlds",  "hospital.sdf")
    slam_cfg  = os.path.join(pkg, "config",  "slam_config.yaml")
    nav2_cfg  = os.path.join(pkg, "config",  "nav2_params.yaml")
    rviz_file = os.path.join(pkg, "rviz",    "phase2_slam.rviz")

    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]), value_type=str
    )
    sim = {"use_sim_time": True}

    # ── Gazebo ────────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen"
    )

    # ── Robot State Publisher ─────────────────────────────────────────
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}, sim],
        output="screen"
    )

    # ── Spawn robot (t+4s) ────────────────────────────────────────────
    spawn = TimerAction(period=4.0, actions=[
        Node(
            package="ros_gz_sim", executable="create",
            arguments=[
                "-name",  "neo_robot",
                "-topic", "robot_description",
                "-x", "-8.0", "-y", "0.0", "-z", "0.50", "-Y", "0.0",
            ],
            output="screen"
        )
    ])

    # ── ROS-GZ Bridge (t+2s) ──────────────────────────────────────────
    bridge = TimerAction(period=2.0, actions=[
        Node(
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
            remappings=[("/world/hospital/model/neo_robot/joint_state", "/joint_states")],
            output="screen"
        )
    ])

    # ── Scan + Odom helpers (t+4.5s) ──────────────────────────────────
    helpers = TimerAction(period=4.5, actions=[
        Node(
            package="neo_assist", executable="scan_frame_fixer",
            parameters=[sim], output="screen"
        ),
        Node(
            package="neo_assist", executable="odom_tf_broadcaster",
            parameters=[sim], output="screen"
        ),
    ])

    # ── SLAM via online_async_launch (t+6s) ───────────────────────────
    slam = TimerAction(period=6.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_pkg, "launch", "online_async_launch.py")
            ),
            launch_arguments={
                "use_sim_time": "true",
                "slam_params_file": slam_cfg,
            }.items()
        )
    ])

    # ── Nav2 (t+8s) — NO amcl/map_server, SLAM provides map ──────────
    nav2 = TimerAction(period=8.0, actions=[
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen",
             parameters=[nav2_cfg, sim]),
        Node(package="nav2_smoother", executable="smoother_server",
             name="smoother_server", output="screen",
             parameters=[nav2_cfg, sim]),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen",
             parameters=[nav2_cfg, sim]),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen",
             parameters=[nav2_cfg, sim]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen",
             parameters=[nav2_cfg, sim]),
        Node(package="nav2_waypoint_follower", executable="waypoint_follower",
             name="waypoint_follower", output="screen",
             parameters=[nav2_cfg, sim]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             name="velocity_smoother", output="screen",
             parameters=[nav2_cfg, sim]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{
                 "use_sim_time": True,
                 "autostart": True,
                 "node_names": [
                     "controller_server", "smoother_server",
                     "planner_server", "behavior_server",
                     "bt_navigator", "waypoint_follower",
                     "velocity_smoother",
                 ]
             }]),
    ])

    # ── RViz (t+7s) ───────────────────────────────────────────────────
    rviz = TimerAction(period=7.0, actions=[
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", rviz_file],
            parameters=[sim], output="screen"
        )
    ])

    # ── Auto Explorer (t+18s — after Nav2 activates) ──────────────────
    explorer = TimerAction(period=18.0, actions=[
        Node(
            package="neo_assist", executable="auto_mapper",
            name="auto_mapper",
            parameters=[sim], output="screen"
        )
    ])

    return LaunchDescription([
        gazebo, rsp, bridge, spawn, helpers,
        slam, nav2, rviz, explorer,
    ])
