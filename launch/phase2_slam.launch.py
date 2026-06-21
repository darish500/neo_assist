"""
NeoAssist Phase 2 — SLAM + Frontier Exploration  (FIXED planner)
=================================================================
KEY FIX vs previous version:
  NavFn was failing to plan to frontier goals with:
    "Failed to create plan with tolerance of: 0.500000"

  Root cause: global costmap had a static_layer loaded from the tiny
  existing map. NavFn saw the area beyond the map boundary as lethal/unknown
  and refused to plan into it — but that's EXACTLY where the frontiers are.

  Fixes applied in nav2_params_phase2.yaml:
    1. Removed static_layer from global costmap — SLAM's live /map topic
       feeds the obstacle_layer directly via OccupancyGrid updates, so we
       don't need a separate static layer during mapping.
    2. use_astar: true — A* handles growing/partial maps much better than
       Dijkstra (NavFn default). It finds paths to frontier edges where
       Dijkstra gives up.
    3. tolerance: 1.5m — frontiers are at map edges, the exact cell may
       not be reachable; 1.5m gives NavFn room to find a nearby endpoint.
    4. movement_time_allowance: 20s — longer paths to far frontiers need
       more time before the progress checker aborts.
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

    pkg      = get_package_share_directory("neo_assist")
    slam_pkg = get_package_share_directory("slam_toolbox")

    urdf     = os.path.join(pkg, "urdf",   "neo_robot.urdf.xacro")
    world    = os.path.join(pkg, "worlds",  "hospital.sdf")
    slam_cfg = os.path.join(pkg, "config",  "slam_config.yaml")
    rviz_cfg = os.path.join(pkg, "rviz",    "phase2_slam.rviz")

    # ── Use PHASE2-SPECIFIC nav2 params (no static_layer, A*, wide tolerance)
    nav2_cfg = os.path.join(pkg, "config",  "nav2_params_phase2.yaml")

    robot_description = ParameterValue(Command(["xacro ", urdf]), value_type=str)
    sim = {"use_sim_time": True}

    # ── 1. Gazebo ─────────────────────────────────────────────────────
    gazebo = ExecuteProcess(cmd=["gz", "sim", "-r", world], output="screen")

    # ── 2. Robot State Publisher ──────────────────────────────────────
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}, sim],
        output="screen"
    )

    # ── 3. ROS-GZ Bridge (t+3s) ───────────────────────────────────────
    bridge = TimerAction(period=3.0, actions=[Node(
        package="ros_gz_bridge", executable="parameter_bridge",
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
    )])

    # ── 4. Spawn robot (t+5s) ─────────────────────────────────────────
    spawn = TimerAction(period=5.0, actions=[Node(
        package="ros_gz_sim", executable="create",
        arguments=["-name","neo_robot","-topic","robot_description",
                   "-x","-8.0","-y","0.0","-z","0.50","-Y","0.0"],
        output="screen"
    )])

    # ── 5. TF helpers (t+5.5s) ────────────────────────────────────────
    helpers = TimerAction(period=5.5, actions=[
        Node(package="neo_assist", executable="scan_frame_fixer",
             parameters=[sim], output="screen"),
        Node(package="neo_assist", executable="odom_tf_broadcaster",
             parameters=[sim], output="screen"),
    ])

    # ── 6. SLAM Toolbox (t+8s) — manages its OWN lifecycle ───────────
    slam = TimerAction(period=8.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_pkg, "launch", "online_async_launch.py")
            ),
            launch_arguments={
                "use_sim_time": "true",
                "slam_params_file": slam_cfg,
            }.items()
        )
        # NO lifecycle_manager here — online_async manages itself
    ])

    # ── 7. Nav2 minimal stack (t+12s) — NO amcl, NO map_server ───────
    nav2 = TimerAction(period=12.0, actions=[
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen",
             parameters=[nav2_cfg, sim],
             remappings=[("cmd_vel", "cmd_vel_nav")]),
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
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             name="velocity_smoother", output="screen",
             parameters=[nav2_cfg, sim],
             remappings=[
                 ("cmd_vel",          "cmd_vel_nav"),
                 ("cmd_vel_smoothed", "cmd_vel"),
             ]),
        Node(package="nav2_lifecycle_manager",
             executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{
                 "use_sim_time": True,
                 "autostart": True,
                 "bond_timeout": 30.0,
                 "node_names": [
                     "controller_server", "smoother_server",
                     "planner_server",    "behavior_server",
                     "bt_navigator",      "velocity_smoother",
                 ],
             }]),
    ])

    # ── 8. RViz (t+10s) ───────────────────────────────────────────────
    rviz = TimerAction(period=10.0, actions=[
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", rviz_cfg],
             parameters=[sim], output="screen")
    ])

    # ── 9. Frontier Explorer (t+35s) ──────────────────────────────────
    # Waits for SLAM to have a meaningful map and Nav2 to be fully active.
    frontier = TimerAction(period=35.0, actions=[
        Node(
            package="frontier_exploration_ros2",
            executable="frontier_explorer",
            name="frontier_explorer",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "robot_radius": 0.45,
                "min_frontier_size": 0.5,
                "travel_point": "closest",
                "visualize": True,
                "mrtsp_solver": "greedy",
                "planner_frequency": 1.0,
            }]
        )
    ])

    return LaunchDescription([
        gazebo, rsp, bridge, spawn, helpers,
        slam, nav2, rviz, frontier,
    ])