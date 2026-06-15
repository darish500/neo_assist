"""
NeoAssist Phase 3 — Map Labelling  (FIXED for ROS2 Jazzy)
==========================================================
Root cause of the original failure:
  nav2_lifecycle_manager fires immediately at launch and tries to
  configure map_server before it has finished its own __init__ /
  "Creating" state.  The fix is a 2-second TimerAction delay so
  map_server reaches "unconfigured" before the manager touches it.

Also added: automatic fallback RViz config so the map always shows
even if phase3_label.rviz doesn't exist yet.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    pkg      = get_package_share_directory("neo_assist")
    map_file = os.path.join(pkg, "maps", "hospital_map.yaml")

    # Use the saved RViz config if it exists, otherwise use a
    # minimal inline config (passed via --display-config isn't
    # supported cleanly so we write a temp file at launch time).
    rviz_file = os.path.join(pkg, "rviz", "phase3_label.rviz")
    if not os.path.exists(rviz_file):
        # Write a minimal working config to /tmp
        rviz_file = "/tmp/phase3_label_auto.rviz"
        _write_fallback_rviz(rviz_file)

    # ── 1. map_server starts immediately ──────────────────────────
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": map_file,
            "use_sim_time": False,
        }]
    )

    # ── 2. lifecycle_manager delayed 2 s ──────────────────────────
    #   Gives map_server time to finish "Creating" and reach
    #   "unconfigured" before the manager sends configure/activate.
    lifecycle_manager = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "autostart": True,
                    "node_names": ["map_server"],
                    # Bond timeout — how long to wait for map_server
                    # to respond to configure before giving up.
                    "bond_timeout": 10.0,
                }]
            )
        ]
    )

    # ── 3. Map labeller ───────────────────────────────────────────
    map_labeller = Node(
        package="neo_assist",
        executable="map_labeller",
        name="map_labeller",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    # ── 4. RViz delayed 3 s (after map_server is active) ─────────
    rviz = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_file],
                output="screen"
            )
        ]
    )

    return LaunchDescription([
        map_server,
        lifecycle_manager,
        map_labeller,
        rviz,
    ])


# ─────────────────────────────────────────────────────────────────
# Fallback RViz config — shows the map + Publish Point tool
# Written to /tmp if rviz/phase3_label.rviz doesn't exist yet.
# ─────────────────────────────────────────────────────────────────
def _write_fallback_rviz(path: str):
    content = """\
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.7
      Class: rviz_map/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 5
        Durability Policy: Transient Local
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Value: true
    - Class: rviz_default_plugins/Axes
      Enabled: true
      Length: 1
      Name: Axes
      Radius: 0.1
      Reference Frame: map
      Value: true
  Enabled: true
  Fixed Frame: map
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/PublishPoint
      Topic: /clicked_point
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Near Clip Distance: 0.01
      Scale: 20
      Target Frame: map
      X: 0
      Y: 0
"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(content)