#!/usr/bin/env python3
"""
cmd_vel_to_sport
================
Pont de SORTIE pour le VRAI go2w : traduit la commande de vitesse de Nav2
en commande de locomotion Unitree (sport API), executee par la policy
embarquee (mode nomad). C'est le maillon "execute navigation" cote robot.

  /cmd_vel_nav (geometry_msgs/Twist, sortie du controller MPPI)
        -> Unitree Move(vx, vy, vyaw)  [api_id 1008, parameter JSON {x,y,z}]
        -> /api/sport/request (unitree_api/Request)  -> robot

Securite :
  - watchdog : si aucune commande recente (> cmd_timeout), on envoie STOPMOVE
    (api_id 1003) pour stopper le robot (perte de Nav2, goal annule, e-stop soft).
  - bornes de vitesse (vx/vy/wz max) re-clampees ici, independamment de MPPI.
  - PAS de mise debout automatique : l'operateur met le robot en BalanceStand
    AVANT (voir doc), pour garder le controle physique du demarrage.

NB : en simulation, c'est mujoco_node qui joue le role du robot (il s'abonne a
/cmd_vel). Ce node-ci ne sert QUE sur le vrai robot.
"""

import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request


ROBOT_SPORT_API_ID_STOPMOVE = 1003
ROBOT_SPORT_API_ID_MOVE     = 1008


class CmdVelToSport(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_sport')

        # ── Parametres ───────────────────────────────────────────────
        self.declare_parameter('in_topic', '/cmd_vel_nav')
        self.declare_parameter('request_topic', '/api/sport/request')
        self.declare_parameter('vx_max', 0.6)    # m/s — bride 1er run reel
        self.declare_parameter('vy_max', 0.4)    # m/s
        self.declare_parameter('wz_max', 1.0)    # rad/s
        self.declare_parameter('cmd_timeout', 0.5)   # s — au-dela : STOPMOVE
        self.declare_parameter('rate_hz', 20.0)      # frequence d'emission Move

        self.in_topic   = self.get_parameter('in_topic').value
        req_topic       = self.get_parameter('request_topic').value
        self.vx_max     = float(self.get_parameter('vx_max').value)
        self.vy_max     = float(self.get_parameter('vy_max').value)
        self.wz_max     = float(self.get_parameter('wz_max').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        rate            = float(self.get_parameter('rate_hz').value)

        self.req_pub = self.create_publisher(Request, req_topic, 10)
        self.create_subscription(Twist, self.in_topic, self._on_cmd, 10)

        self._last_cmd = (0.0, 0.0, 0.0)
        self._last_stamp = None
        self._stopped = True   # etat connu : robot a l'arret

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'cmd_vel_to_sport pret : {self.in_topic} (Twist) -> {req_topic} '
            f'(Unitree Move). Bornes vx<={self.vx_max} vy<={self.vy_max} '
            f'wz<={self.wz_max}. ATTENTION : mettre le robot en BalanceStand avant.')

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _on_cmd(self, msg: Twist):
        vx = self._clamp(msg.linear.x,  -self.vx_max, self.vx_max)
        vy = self._clamp(msg.linear.y,  -self.vy_max, self.vy_max)
        wz = self._clamp(msg.angular.z, -self.wz_max, self.wz_max)
        self._last_cmd = (vx, vy, wz)
        self._last_stamp = self.get_clock().now()

    def _send_move(self, vx, vy, vyaw):
        req = Request()
        req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE
        req.parameter = json.dumps({'x': vx, 'y': vy, 'z': vyaw})
        self.req_pub.publish(req)
        self._stopped = False

    def _send_stop(self):
        if self._stopped:
            return
        req = Request()
        req.header.identity.api_id = ROBOT_SPORT_API_ID_STOPMOVE
        self.req_pub.publish(req)
        self._stopped = True
        self.get_logger().warn('STOPMOVE envoye (commande absente/perimee).')

    def _tick(self):
        now = self.get_clock().now()
        stale = (self._last_stamp is None or
                 (now - self._last_stamp).nanoseconds * 1e-9 > self.cmd_timeout)
        if stale:
            self._send_stop()
            return
        self._send_move(*self._last_cmd)


def main():
    rclpy.init()
    node = CmdVelToSport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
