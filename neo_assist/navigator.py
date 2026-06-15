#!/usr/bin/env python3
"""
navigator.py
============
Sends the robot to named locations from locations.yaml.

USAGE:
  ros2 run neo_assist navigator
  Then type: go to pharmacy
         or: go to reception
         or: list
         or: quit
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import yaml
import os
import math
import threading


class Navigator(Node):
    def __init__(self):
        super().__init__("navigator")

        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Load locations
        locations_file = os.path.expanduser(
            "~/ros2_ws/src/neo_assist/maps/locations.yaml"
        )
        with open(locations_file, "r") as f:
            self.locations = yaml.safe_load(f)

        self.get_logger().info(f"Loaded {len(self.locations)} locations:")
        for name in self.locations:
            self.get_logger().info(f"  - {name}")

        print("\n" + "="*50)
        print("  NeoAssist Navigator")
        print("="*50)
        print("  Commands:")
        print("  - 'list'              → show all rooms")
        print("  - 'go to <room>'      → navigate to room")
        print("  - 'quit'              → exit")
        print("="*50)

        # Run input loop in separate thread
        threading.Thread(target=self.input_loop, daemon=True).start()

    def input_loop(self):
        while rclpy.ok():
            try:
                cmd = input("\n> ").strip().lower()
            except EOFError:
                break

            if cmd == "quit":
                rclpy.shutdown()
                break
            elif cmd == "list":
                print("\nAvailable locations:")
                for name, data in self.locations.items():
                    print(f"  {name:20s} → ({data['x']:.2f}, {data['y']:.2f})")
            elif cmd.startswith("go to "):
                room = cmd[6:].strip()
                if room in self.locations:
                    self.navigate_to(room)
                else:
                    print(f"  ❌ Unknown room: '{room}'")
                    print(f"  Try: {list(self.locations.keys())}")
            else:
                print("  Unknown command. Try 'list' or 'go to <room>'")

    def navigate_to(self, room_name: str):
        data = self.locations[room_name]
        print(f"\n🚀 Navigating to {room_name} ({data['x']:.2f}, {data['y']:.2f})...")

        # Wait for Nav2 action server
        if not self._client.wait_for_server(timeout_sec=5.0):
            print("  ❌ Nav2 action server not available!")
            return

        # Build goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(data["x"])
        goal.pose.pose.position.y = float(data["y"])
        goal.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        yaw = float(data.get("yaw", 0.0))
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        # Send goal
        future = self._client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(
            lambda f: self.goal_response_callback(f, room_name)
        )

    def feedback_callback(self, feedback):
        dist = feedback.feedback.distance_remaining
        print(f"  📍 Distance remaining: {dist:.2f}m", end="\r")

    def goal_response_callback(self, future, room_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print(f"  ❌ Goal rejected!")
            return
        print(f"  ✅ Goal accepted — robot is moving to {room_name}...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self.result_callback(f, room_name)
        )

    def result_callback(self, future, room_name):
        print(f"\n  🏁 Arrived at {room_name}!")


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
