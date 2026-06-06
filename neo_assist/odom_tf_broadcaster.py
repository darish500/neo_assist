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

        # 20 Hz timer — keeps TF cache populated between odom callbacks
        # This prevents the "timestamp earlier than cache" error
        self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("OdomTfBroadcaster ready — 20 Hz TF publishing")

    def timer_callback(self):
        """Publish TF at 20 Hz using latest known pose."""
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id  = "base_footprint"
        t.transform.translation.x = self._latest_x
        t.transform.translation.y = self._latest_y
        t.transform.translation.z = self._latest_z
        t.transform.rotation.x = self._latest_qx
        t.transform.rotation.y = self._latest_qy
        t.transform.rotation.z = self._latest_qz
        t.transform.rotation.w = self._latest_qw
        self.tf_broadcaster.sendTransform(t)

    def odom_callback(self, msg: Odometry):
        """Update stored pose when new odom arrives."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._latest_x  = p.x
        self._latest_y  = p.y
        self._latest_z  = p.z
        self._latest_qx = q.x
        self._latest_qy = q.y
        self._latest_qz = q.z
        self._latest_qw = q.w

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
