#!/usr/bin/env python3
"""
sportmode_to_odom
=================
Pont d'ENTREE pour le VRAI go2w : transforme l'etat haut-niveau Unitree en
odometrie ROS standard. C'est le maillon "robot knows where it is" cote robot.

  /sportmodestate (unitree_go/SportModeState)
      - position[3]          : pose estimee (x,y,z) en frame monde/odom
      - imu_state.quaternion : orientation [w,x,y,z]  (convention Unitree)
      - velocity[3]          : vitesse lineaire corps (vx,vy,vz)
      - yaw_speed            : vitesse de lacet
  ->  /odom (nav_msgs/Odometry, frame odom -> base_link)
  ->  TF  odom -> base_link

Remplace la TF ground-truth que publiait mujoco_node en simulation.
Pas de carte globale ici : on navigue en frame `odom`. Une TF statique
identite `map -> odom` est publiee a part (launch) pour satisfaire Nav2.

ATTENTION conventions a verifier sur le robot avant l'experience :
  - quaternion Unitree suppose [w,x,y,z] -> ROS [x,y,z,w] (remappe ci-dessous).
  - velocity supposee en repere CORPS (twist enfant). Si Unitree la donne en
    repere monde, mettre `velocity_in_body=False`.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class SportModeToOdom(Node):

    def __init__(self):
        super().__init__('sportmode_to_odom')

        self.declare_parameter('state_topic', '/sportmodestate')  # ou /lf/sportmodestate
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('velocity_in_body', True)

        state_topic     = self.get_parameter('state_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        # Etat robot publie en best-effort par le pont DDS Unitree
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(SportModeState, state_topic, self._on_state, qos)
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)
        self.tf_bc = TransformBroadcaster(self)

        self.get_logger().info(
            f'sportmode_to_odom pret : {state_topic} -> /odom + TF '
            f'{self.odom_frame}->{self.base_frame}')

    def _on_state(self, msg: SportModeState):
        stamp = self.get_clock().now().to_msg()
        px, py, pz = msg.position[0], msg.position[1], msg.position[2]
        # quaternion Unitree [w,x,y,z] -> ROS [x,y,z,w]
        qw, qx, qy, qz = (msg.imu_state.quaternion[0],
                          msg.imu_state.quaternion[1],
                          msg.imu_state.quaternion[2],
                          msg.imu_state.quaternion[3])

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(px)
        odom.pose.pose.position.y = float(py)
        odom.pose.pose.position.z = float(pz)
        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])
        odom.twist.twist.angular.z = float(msg.yaw_speed)
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = float(px)
            tf.transform.translation.y = float(py)
            tf.transform.translation.z = float(pz)
            tf.transform.rotation.x = float(qx)
            tf.transform.rotation.y = float(qy)
            tf.transform.rotation.z = float(qz)
            tf.transform.rotation.w = float(qw)
            self.tf_bc.sendTransform(tf)


def main():
    rclpy.init()
    node = SportModeToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
