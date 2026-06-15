#!/usr/bin/env python3
"""
auto_mapper.py — Hospital Autonomous Explorer
=============================================
Visits all hospital rooms while SLAM builds the map.
Uses async callbacks (same pattern as warehouse mission_controller).
Saves map automatically when done.
"""
import json, math, os, subprocess
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class HospitalExplorer(Node):

    # Hospital waypoints covering all rooms
    # (name, x, y, yaw_deg)
    WAYPOINTS = [
        ("corridor_west",    -5.0,  0.0,   0.0),
        ("entrance_lobby",   -5.0,  5.0,  90.0),
        ("entrance_deep",    -8.0,  6.0, 180.0),
        ("corridor_centre",   0.0,  0.0,   0.0),
        ("reception_entry",   3.0,  3.0,  90.0),
        ("reception_deep",    6.0,  6.0,   0.0),
        ("corridor_east",     5.0,  0.0,   0.0),
        ("ward_b_entry",      5.0, -3.0, -90.0),
        ("ward_b_deep",       7.0, -6.0, -90.0),
        ("corridor_south",    0.0, -3.0, 180.0),
        ("ward_a_entry",     -5.0, -3.0, -90.0),
        ("ward_a_deep",      -7.0, -6.0, -90.0),
        ("lab_entry",        -5.0, -6.0, -90.0),
        ("toilet_entry",      3.0, -6.0, -90.0),
        ("corridor_return",   0.0,  0.0, 180.0),
        ("loop_closure",     -8.0,  0.0, 180.0),
    ]

    def __init__(self):
        super().__init__("auto_mapper")
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._status_pub = self.create_publisher(String, "/explorer_status", 10)
        self._wp_index = 0
        self._nav_check_count = 0

        self.get_logger().info("🤖 Hospital Explorer ready. Waiting for Nav2...")
        self.create_timer(2.0, self._check_nav_ready)

    def _check_nav_ready(self):
        self._nav_check_count += 1
        if self._nav.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("✅ Nav2 ready! Starting hospital exploration.")
            self.destroy_timer(self._check_nav_ready)
            self._visit_next()
        elif self._nav_check_count % 5 == 0:
            self.get_logger().info(
                f"⏳ Waiting for Nav2... ({self._nav_check_count*2}s)")

    def _visit_next(self):
        if self._wp_index >= len(self.WAYPOINTS):
            self.get_logger().info("\n🎉 All rooms explored! Saving map...")
            self._save_map()
            return

        name, x, y, yaw = self.WAYPOINTS[self._wp_index]
        self._wp_index += 1
        total = len(self.WAYPOINTS)

        self.get_logger().info(
            f"\n[{self._wp_index}/{total}] → {name} ({x}, {y})")
        self._publish_status(f"EXPLORING_{name.upper()}", self._wp_index, total)
        self._navigate_to(x, y, yaw, name)

    def _navigate_to(self, x, y, yaw_deg, name):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        yaw = math.radians(yaw_deg)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        future = self._nav.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._goal_response(f, name))

    def _goal_response(self, future, name):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn(f"⚠️  Goal rejected for {name} — skipping")
            self.create_timer(1.0, lambda: self._visit_next())
            return
        self.get_logger().info(f"  ✅ Moving to {name}...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._nav_result(f, name))

    def _nav_result(self, future, name):
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"  🏁 Reached {name}! Pausing 2s...")
        else:
            self.get_logger().warn(
                f"  ⚠️  Nav failed for {name} (status={result.status}) — continuing")
        self.create_timer(2.0, lambda: self._visit_next())

    def _save_map(self):
        path = os.path.expanduser(
            "~/ros2_ws/src/neo_assist/maps/hospital_map")
        self.get_logger().info(f"💾 Saving map to {path}...")
        result = subprocess.run(
            ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", path],
            capture_output=True, text=True, timeout=30
        )
        if "saved" in result.stdout.lower():
            self.get_logger().info("✅ Map saved!")
        else:
            self.get_logger().error(f"❌ Map save failed: {result.stderr}")

    def _publish_status(self, state, current, total):
        msg = String()
        msg.data = json.dumps({
            "state": state,
            "waypoint": current,
            "total": total
        })
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HospitalExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted — saving map...")
        node._save_map()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
