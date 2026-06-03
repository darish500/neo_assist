"""
NeoAssist Phase 1 Launch — Spawn robot in hospital world
=========================================================
What this does:
  1. Starts Gazebo Harmonic with the hospital world
  2. Processes the URDF xacro into XML
  3. Spawns the robot at the entrance (x=-8, y=0)
  4. Starts robot_state_publisher (broadcasts TF frames)
  5. Starts ros_gz_bridge (connects Gazebo topics to ROS2)
  6. Opens RViz for visualization

Run with:
  ros2 launch neo_assist phase1_spawn.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ── Path helpers ─────────────────────────────────
    pkg_dir = get_package_share_directory('neo_assist')
    urdf_file  = os.path.join(pkg_dir, 'urdf', 'neo_robot.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'hospital.sdf')

    # ── Process xacro → URDF XML at launch time ──────
    # Command() runs xacro on the file and passes the result
    # to robot_state_publisher as the robot_description param
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # ── 1. Gazebo Harmonic ────────────────────────────
    # gz sim -r = run simulation immediately (no pause)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # ── 2. Robot State Publisher ─────────────────────
    # Reads robot_description and publishes TF frames
    # (where each link is relative to each other)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,     # sync to Gazebo clock
        }]
    )

    # ── 3. Spawn robot into Gazebo ───────────────────
    # Reads robot_description from the ROS2 parameter server
    # and tells Gazebo to place it at x=-8 y=0 (entrance)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_neo_robot',
        arguments=[
            '-name',  'neo_robot',
            '-topic', 'robot_description',   # reads from param server
            '-x',     '-8.0',                # entrance x position
            '-y',     '0.0',                 # centre of corridor
            '-z',     '0.12',                # just above ground
            '-Y',     '0.0',                 # facing east (+X)
        ],
        output='screen'
    )

    # ── 4. ROS-Gazebo Bridge ─────────────────────────
    # Bridges Gazebo internal topics ↔ ROS2 topics
    # Format: gz_topic@ros_msg_type@gz_msg_type
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Clock — lets ROS2 nodes use Gazebo sim time
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # LiDAR scan — from Gazebo sensor to ROS2 LaserScan
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Camera image — raw RGB frames
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # Odometry — Gazebo diff-drive → ROS2 odom
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Velocity commands — ROS2 → Gazebo actuator
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Joint states — for wheel visualization in RViz
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # TF from Gazebo (odom → base_footprint)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen'
    )

    # ── 5. RViz2 ─────────────────────────────────────
    # Opens the visualization tool — we'll configure it
    # to show the robot model and LiDAR scan
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        rviz,
    ])
