#!/usr/bin/env python3
"""
map_labeller.py
===============
HOW IT WORKS:
  1. Launches RViz showing your saved hospital map
  2. You click a location in RViz using "Publish Point" tool
  3. The coordinates appear in this terminal
  4. You type a name for that location (e.g. "lab", "ward_a")
  5. Repeat for every room
  6. All labels saved to locations.yaml

RUN WITH:
  python3 ~/ros2_ws/src/neo_assist/neo_assist/map_labeller.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import yaml
import os
import sys


class MapLabeller(Node):
    def __init__(self):
        super().__init__("map_labeller")

        # Where labels will be saved
        self.output_file = os.path.expanduser(
            "~/ros2_ws/src/neo_assist/maps/locations.yaml"
        )

        # Load existing labels if file exists
        if os.path.exists(self.output_file):
            with open(self.output_file, "r") as f:
                self.locations = yaml.safe_load(f) or {}
            print(f"\n📂 Loaded existing labels: {list(self.locations.keys())}")
        else:
            self.locations = {}

        # Subscribe to RViz "Publish Point" clicks
        self.sub = self.create_subscription(
            PointStamped,
            "/clicked_point",      # RViz publishes here when you use Publish Point tool
            self.point_callback,
            10
        )

        print("\n" + "="*55)
        print("  NeoAssist Map Labeller")
        print("="*55)
        print("  In RViz:")
        print("  1. Click the TARGET icon in the toolbar")
        print("     (it looks like a crosshair/bullseye)")
        print("  2. Click on a room centre in the map")
        print("  3. Come back here and type the room name")
        print("="*55)
        print("\n  Rooms to label:")
        print("  - entrance     (west end of hospital)")
        print("  - corridor     (centre hallway)")
        print("  - reception    (north-east room)")
        print("  - ward_a       (south-west room)")
        print("  - ward_b       (south-east room)")
        print("  - lab          (south-west corner)")
        print("  - toilet       (south-east corner)")
        print("="*55)
        print("\n  Waiting for you to click in RViz...\n")

        self.pending_point = None

    def point_callback(self, msg: PointStamped):
        """Called when you click in RViz with Publish Point tool."""
        x = round(msg.point.x, 3)
        y = round(msg.point.y, 3)

        print(f"\n📍 You clicked: x={x}, y={y}")
        print("   Type the room name and press Enter")
        print("   (or press Enter to skip this point):")

        name = input("   Room name: ").strip().lower()

        if name:
            # Save with position and a default orientation (facing east)
            self.locations[name] = {
                "x": x,
                "y": y,
                "z": 0.0,
                "yaw": 0.0,   # facing east — change if needed
                "description": name.replace("_", " ").title()
            }

            # Write to file immediately
            with open(self.output_file, "w") as f:
                yaml.dump(self.locations, f, default_flow_style=False)

            print(f"   ✅ Saved '{name}' at ({x}, {y})")
            print(f"   📁 File: {self.output_file}")
            print(f"\n   Labelled so far: {list(self.locations.keys())}")
            print("\n   Click another room in RViz (or Ctrl+C to finish)...")
        else:
            print("   ⏭️  Skipped")
            print("\n   Click another room in RViz...")


def main():
    rclpy.init()
    node = MapLabeller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n\n{'='*55}")
        print("  Labelling complete!")
        print(f"  Saved {len(node.locations)} locations:")
        for name, data in node.locations.items():
            print(f"    {name}: ({data['x']}, {data['y']})")
        print(f"{'='*55}\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
