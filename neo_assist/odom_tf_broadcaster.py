#!/usr/bin/env python3
"""
odom_tf_broadcaster.py
Publishes odom -> base_footprint TF.
Also runs a 20 Hz timer to keep TF cache populated
even when odom messages are slow.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__("odom_tf_broadcaster")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/odom_corrected", 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # Store latest pose so timer can republish it
        self._latest_x = 0.0
        self._latest_y = 0.0
        self._latest_z = 0.0
        self._latest_qx = 0.0
        self._latest_qy = 0.0
        self._latest_qz = 0.0
        self._latest_qw = 1.0

        self.get_logger().info("OdomTfBroadcaster ready — TF published from real odom stamps")

    def odom_callback(self, msg: Odometry):
        """Publish TF and republish odom, both using the REAL odom timestamp
        (not get_clock().now()) so the scan and TF stay in sync."""
        t = TransformStamped()
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id  = "base_footprint"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # Also republish corrected odom for Nav2/SLAM
        msg.header.frame_id = "odom"
        msg.child_frame_id  = "base_footprint"
        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
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
