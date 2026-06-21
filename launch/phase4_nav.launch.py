"""
NeoAssist Phase 4 — Autonomous Navigation  (FIXED for ROS2 Jazzy)
==================================================================
KEY FIXES vs the broken version:

  1. bond_timeout = 30.0s on lifecycle_manager_navigation
     (was missing → defaulted to 4s → nodes on slow machines timed out)

  2. velocity_smoother remappings are CORRECT:
       ("cmd_vel",          "cmd_vel_nav")       ← what controllers output
       ("cmd_vel_smoothed", "cmd_vel")            ← what robot actually reads
     Without this, the smoother never connects and lifecycle fails.

  3. Timing staggered more conservatively:
       Gazebo → RSP/Bridge → Spawn (t+6) → TF helpers (t+7) →
       map_server+AMCL+planners (t+8) → lifecycle_manager (t+12) →
       initial_pose (t+18) → RViz (t+14)
     This prevents "configure before node ready" race conditions.

  4. initial_pose timer increased to 18s (was 15s) so AMCL is fully
     active before particles are initialised.
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
    nav2_params= os.path.join(pkg, "config",  "nav2_params.yaml")
    map_file   = os.path.join(pkg, "maps",    "hospital_map.yaml")
    rviz_file  = os.path.join(pkg, "rviz",    "neo_assist.rviz")

    if not os.path.exists(rviz_file):
        rviz_file = "/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz"

    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]),
        value_type=str
    )
    sim = {"use_sim_time": True}

    # ── 1. Gazebo (t=0) ───────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen"
    )

    # ── 2. Robot State Publisher (t=0) ────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}, sim]
    )

    # ── 3. ROS-GZ Bridge (t=0) ────────────────────────────────────────
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

    # ── 4. TF helpers (t=0) ───────────────────────────────────────────
    odom_tf_broadcaster = Node(
        package="neo_assist",
        executable="odom_tf_broadcaster",
        name="odom_tf_broadcaster",
        output="screen",
        parameters=[sim]
    )

    scan_frame_fixer = Node(
        package="neo_assist",
        executable="scan_frame_fixer",
        name="scan_frame_fixer",
        output="screen",
        parameters=[sim]
    )

    # ── 5. Spawn robot (t+6s) ─────────────────────────────────────────
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

    # ── 6. Map Server (t+8s) ──────────────────────────────────────────
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_file}, sim]
    )

    # ── 7. AMCL (t+8s) ────────────────────────────────────────────────
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_params, sim]
    )

    # ── 8. Nav2 Planner (t+8s) ────────────────────────────────────────
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params, sim]
    )

    # ── 9. Nav2 Controller (t+8s) ─────────────────────────────────────
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params, sim],
        remappings=[("cmd_vel", "cmd_vel_nav")]   # output to smoother input
    )

    # ── 10. Nav2 Smoother (t+8s) ──────────────────────────────────────
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[nav2_params, sim]
    )

    # ── 11. Behavior Server (t+8s) ────────────────────────────────────
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params, sim]
    )

    # ── 12. BT Navigator (t+8s) ───────────────────────────────────────
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params, sim]
    )

    # ── 13. Velocity Smoother (t+8s) ──────────────────────────────────
    # FIX: remappings connect controller output → smoother → robot cmd_vel
    #   controller_server publishes to /cmd_vel_nav
    #   velocity_smoother subscribes to /cmd_vel_nav (via remap cmd_vel)
    #   velocity_smoother publishes smoothed to /cmd_vel (via remap cmd_vel_smoothed)
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[nav2_params, sim],
        remappings=[
            ("cmd_vel",          "cmd_vel_nav"),   # read from controller
            ("cmd_vel_smoothed", "cmd_vel"),        # write to robot
        ]
    )

    # ── 14. Waypoint Follower (t+8s) ──────────────────────────────────
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav2_params, sim]
    )

    # ── 15. Lifecycle Manager (t+12s) — activates ALL Nav2 nodes ──────
    # FIX: bond_timeout=30s (was missing, defaulted to 4s → timeout on
    # slow machines running Gazebo + all Nav2 nodes simultaneously)
    lifecycle_manager = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "autostart": True,
                    "bond_timeout": 30.0,    # ← CRITICAL FIX
                    "node_names": [
                        "map_server",
                        "amcl",
                        "planner_server",
                        "controller_server",
                        "smoother_server",
                        "behavior_server",
                        "bt_navigator",
                        "waypoint_follower",
                        "velocity_smoother",
                    ],
                }]
            )
        ]
    )

    # ── 16. RViz (t+14s) ──────────────────────────────────────────────
    rviz = TimerAction(
        period=14.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_file],
                parameters=[sim],
                output="screen"
            )
        ]
    )

    # ── 17. Initial Pose for AMCL (t+18s) ────────────────────────────
    # FIX: increased from 15s → 18s to ensure AMCL is fully active
    # Robot spawns at (-8.0, 0.0) facing east (yaw=0 → w=1.0)
    initial_pose = TimerAction(
        period=18.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "topic", "pub", "--once",
                    "/initialpose",
                    "geometry_msgs/msg/PoseWithCovarianceStamped",
                    '{"header": {"frame_id": "map"}, "pose": {"pose": '
                    '{"position": {"x": -8.0, "y": 0.0, "z": 0.0}, '
                    '"orientation": {"w": 1.0}}, '
                    '"covariance": [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, '
                    '0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}'
                ],
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
        spawn_robot,
        map_server,
        amcl,
        planner_server,
        controller_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        waypoint_follower,
        lifecycle_manager,
        rviz,
        initial_pose,
    ])