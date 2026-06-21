#!/usr/bin/env python3
"""
scan_frame_fixer.py
Relays /scan -> /scan_fixed with:
  1. frame_id corrected to lidar_link
  2. timestamp updated to current sim time (eliminates TF cache miss)
  3. QoS converted BEST_EFFORT -> RELIABLE
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class ScanFrameFixer(Node):
    def __init__(self):
        super().__init__("scan_frame_fixer")

        gz_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        ros_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.pub = self.create_publisher(LaserScan, "/scan_fixed", ros_qos)
        self.sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, gz_qos
        )
        self.get_logger().info(
            "ScanFrameFixer ready — /scan -> /scan_fixed"
        )

    def scan_callback(self, msg: LaserScan):
        # Fix frame_id so TF lookup finds lidar_link
        msg.header.frame_id = "lidar_link"
        # NOTE: keep Gazebo's real timestamp here. Rewriting it to "now"
        # desyncs the scan from the TF pose at capture time, which corrupts
        # costmap obstacle placement during rotation — this was the root
        # cause of the "turns a bit then collides" bug.
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameFixer()
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
