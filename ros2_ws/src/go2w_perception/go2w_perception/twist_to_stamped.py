#!/usr/bin/env python3
"""
twist_to_stamped
================
Mini relais : Nav2 (Humble) publie la commande en geometry_msgs/Twist, mais le
mujoco_node du go2w attend du geometry_msgs/TwistStamped.

  /cmd_vel_nav (Twist)  ──>  /cmd_vel (TwistStamped, frame base_link)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistToStamped(Node):
    def __init__(self):
        super().__init__('twist_to_stamped')
        self.declare_parameter('in_topic', '/cmd_vel_nav')
        self.declare_parameter('out_topic', '/cmd_vel')
        self.declare_parameter('frame_id', 'base_link')
        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(TwistStamped, out_topic, 10)
        self.create_subscription(Twist, in_topic, self._cb, 10)
        self.get_logger().info(f'relais {in_topic} (Twist) -> {out_topic} (TwistStamped)')

    def _cb(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TwistToStamped()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
