"""
NeoAssist Phase 4 — Autonomous Navigation
==========================================
Launches:
  - Gazebo with hospital world
  - Robot State Publisher
  - ROS-GZ Bridge
  - odom_tf_broadcaster
  - scan_frame_fixer
  - Nav2 full stack (AMCL + planner + controller)
  - RViz with Nav2 view
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg       = get_package_share_directory("neo_assist")
    urdf_file = os.path.join(pkg, "urdf",   "neo_robot.urdf.xacro")
    world_file= os.path.join(pkg, "worlds",  "hospital.sdf")
    nav2_params= os.path.join(pkg, "config", "nav2_params.yaml")
    map_file  = os.path.join(pkg, "maps",    "hospital_map.yaml")
    rviz_file = os.path.join(pkg, "rviz",    "phase4_nav.rviz")

    # Use nav2 default rviz if ours doesn't exist yet
    if not os.path.exists(rviz_file):
        rviz_file = "/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz"

    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]),
        value_type=str
    )

    # ── Gazebo ────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen"
    )

    # ── Robot State Publisher ─────────────────────────────────────
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

    # ── ROS-GZ Bridge ─────────────────────────────────────────────
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

    # ── Custom TF + Scan nodes ────────────────────────────────────
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

    # ── Spawn Robot ───────────────────────────────────────────────
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

    # ── Map Server ────────────────────────────────────────────────
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": map_file,
            "use_sim_time": True,
        }]
    )

    # ── AMCL ─────────────────────────────────────────────────────
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_params]
    )

    # ── Nav2 Planner ──────────────────────────────────────────────
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params]
    )

    # ── Nav2 Controller ───────────────────────────────────────────
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params],
        remappings=[("cmd_vel", "cmd_vel")]
    )

    # ── Nav2 Smoother ─────────────────────────────────────────────
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[nav2_params]
    )

    # ── Nav2 Behaviors ────────────────────────────────────────────
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params]
    )

    # ── BT Navigator ─────────────────────────────────────────────
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params]
    )

    # ── Velocity Smoother ─────────────────────────────────────────
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[nav2_params],
        remappings=[
            ("cmd_vel", "cmd_vel_smoothed"),
            ("cmd_vel_smoothed", "cmd_vel"),
        ]
    )

    # ── Waypoint Follower ─────────────────────────────────────────
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav2_params]
    )

    # ── Lifecycle Manager — activates ALL Nav2 nodes ──────────────
    lifecycle_manager = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "autostart": True,
                    "bond_timeout": 20.0,
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

    # ── RViz ──────────────────────────────────────────────────────
    rviz = TimerAction(
        period=10.0,
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


    # Auto-publish initial pose so AMCL activates without manual RViz click
    # Robot spawns at (-8, 0) facing east
    initial_pose = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "topic", "pub", "--once",
                    "/initialpose",
                    "geometry_msgs/msg/PoseWithCovarianceStamped",
                    '{"header": {"frame_id": "map"}, "pose": {"pose": {"position": {"x": -8.0, "y": 0.0, "z": 0.0}, "orientation": {"w": 1.0}}, "covariance": [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.068]}}'
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
        map_server,
        amcl,
        planner_server,
        controller_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        spawn_robot,
        lifecycle_manager,
        initial_pose,
        rviz,
    ])


def publish_initial_pose():
    """Publishes initial pose so AMCL activates without manual RViz click"""
    import subprocess, time
    time.sleep(12)  # wait for AMCL to be active
    subprocess.run([
        "ros2", "topic", "pub", "--once",
        "/initialpose",
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        '{"header": {"frame_id": "map"}, "pose": {"pose": {"position": {"x": -8.0, "y": 0.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}, "covariance": [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.06853891945200942]}}'
    ])
