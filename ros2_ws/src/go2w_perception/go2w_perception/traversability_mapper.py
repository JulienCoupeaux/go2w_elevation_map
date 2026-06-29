#!/usr/bin/env python3
"""
traversability_mapper
=====================
Construit une carte de traversabilite 2.5D a partir du /pointcloud LiDAR du go2w,
et l'expose a Nav2 sous trois formes :

  1. nav_msgs/OccupancyGrid   sur /traversability/costmap   (cout gradue 0-100)
       -> consomme par un StaticLayer du costmap Nav2 (terrain franchissable gradue)
  2. sensor_msgs/PointCloud2  sur /traversability/obstacles (cellules letales)
       -> consomme par un ObstacleLayer du costmap Nav2 (vrais obstacles)
  3. grid_map_msgs/GridMap    sur /traversability/grid_map  (elevation + traversabilite)
       -> jolie visu 2.5D dans RViz

Principe : on accumule une elevation map (z-max par cellule) en frame `odom`, puis
on classe chaque cellule par la HAUTEUR DE MARCHE locale (max |dz| vers les voisins).
Tune pour la scene go2w : bordures ~8 cm = franchissables (cout faible), boites
>= 30 cm = letales.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, Float32MultiArray, MultiArrayDimension
from grid_map_msgs.msg import GridMap

from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException


# ── Grille (frame odom) ───────────────────────────────────────────────
RESOLUTION = 0.10          # m / cellule
ORIGIN_X   = -8.0          # coin de la grille en odom (x mini)
ORIGIN_Y   = -10.0         # coin de la grille en odom (y mini)
SIZE_X     = 20.0          # m (etendue x)
SIZE_Y     = 20.0          # m (etendue y)

# ── Filtres pointcloud ────────────────────────────────────────────────
Z_MIN = -0.5               # m (en odom) — on jette le bruit sous le sol
Z_MAX =  2.5               # m — au-dessus du robot, ignore

# ── Classification traversabilite (pente sur fenetre robot) ───────────
# On juge la franchissabilite par la PENTE GLOBALE du terrain sur une
# fenetre ~ empattement du robot, et NON par la marche vers le voisin
# immediat. La metrique "marche locale" condamne a tort tout escalier :
# chaque contremarche est une falaise d'une cellule, alors que la pente
# moyenne d'un escalier reste montable (et le vrai go2w grimpe en mode
# nomad). Mesurer la pente sur une fenetre repartit aussi le denivele sur
# la distance horizontale -> robuste a l'occlusion des marches du fond.
SLOPE_WINDOW    = 0.30    # m — demi-fenetre du calcul de pente (~ empattement)
SLOPE_FLAT_DEG  = 15.0    # deg — sous ce seuil : sol plat, cout 0
SLOPE_CLIMB_DEG = 42.0    # deg — limite confortable d'escalade (escaliers) : reste bon marche
COST_AT_CLIMB   = 25      # cout a SLOPE_CLIMB_DEG (franchissable, faiblement penalise)
SLOPE_MAX_DEG   = 55.0    # deg — pente max absolue ; au-dela : letal
ABS_STEP_LETHAL = 0.55    # m — garde-fou : falaise verticale (mur) > ca = letal

PUBLISH_HZ = 5.0


class TraversabilityMapper(Node):

    def __init__(self):
        super().__init__('traversability_mapper')

        self.NX = int(round(SIZE_X / RESOLUTION))   # cellules en x
        self.NY = int(round(SIZE_Y / RESOLUTION))   # cellules en y

        # Elevation accumulee (z-max). NaN = inconnu. Shape (NY, NX) -> [j, i].
        self.elevation = np.full((self.NY, self.NX), np.nan, dtype=np.float32)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(PointCloud2, '/pointcloud',
                                 self._on_pointcloud, 10)

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.costmap_pub  = self.create_publisher(
            OccupancyGrid, '/traversability/costmap', latched)
        self.obstacle_pub = self.create_publisher(
            PointCloud2, '/traversability/obstacles', 10)
        self.gridmap_pub  = self.create_publisher(
            GridMap, '/traversability/grid_map', 1)

        self._latest = None
        self.create_timer(1.0 / PUBLISH_HZ, self._update)
        self.get_logger().info(
            f'traversability_mapper pret — grille {self.NX}x{self.NY} '
            f'@ {RESOLUTION} m en frame odom')

    # ── Reception pointcloud ──────────────────────────────────────────
    def _on_pointcloud(self, msg: PointCloud2):
        self._latest = msg

    @staticmethod
    def _parse_xyz(msg: PointCloud2):
        n = msg.width * msg.height
        if n == 0:
            return np.zeros((0, 3), dtype=np.float32)
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
        x = raw[:, 0:4].copy().view(np.float32).flatten()
        y = raw[:, 4:8].copy().view(np.float32).flatten()
        z = raw[:, 8:12].copy().view(np.float32).flatten()
        xyz = np.stack([x, y, z], axis=1)
        return xyz[np.isfinite(xyz).all(axis=1)]

    def _lookup_cloud_to_odom(self, cloud_frame):
        """Retourne (t[3], R[3,3]) de `cloud_frame` vers odom, ou None.

        On utilise la frame du nuage (msg.header.frame_id) et non 'base_link'
        en dur : en sim le nuage est en base_link, sur le vrai robot il est dans
        la frame du LiDAR (ex. 'hesai_lidar'). La TF odom<-base_link<-lidar est
        composee automatiquement par tf2 (cf TF statique base_link->lidar)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', cloud_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException,
                ExtrapolationException):
            return None
        q = tf.transform.rotation
        tr = tf.transform.translation
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        R = np.array([
            [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
            [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
            [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
        ], dtype=np.float64)
        return np.array([tr.x, tr.y, tr.z], dtype=np.float64), R

    # ── Boucle principale ─────────────────────────────────────────────
    def _update(self):
        if self._latest is None:
            return
        msg = self._latest

        cloud_frame = msg.header.frame_id or 'base_link'
        tf = self._lookup_cloud_to_odom(cloud_frame)
        if tf is None:
            return
        t_vec, R = tf

        pts_cloud = self._parse_xyz(msg)
        if len(pts_cloud) == 0:
            return

        # frame du LiDAR (ou base_link en sim) -> odom
        pts = (R @ pts_cloud.T).T + t_vec

        # Filtre vertical
        m = (pts[:, 2] > Z_MIN) & (pts[:, 2] < Z_MAX)
        pts = pts[m]
        if len(pts) == 0:
            return

        # Indices cellules
        ix = np.floor((pts[:, 0] - ORIGIN_X) / RESOLUTION).astype(np.int64)
        iy = np.floor((pts[:, 1] - ORIGIN_Y) / RESOLUTION).astype(np.int64)
        inside = (ix >= 0) & (ix < self.NX) & (iy >= 0) & (iy < self.NY)
        ix, iy = ix[inside], iy[inside]
        z = pts[inside, 2].astype(np.float32)
        if len(z) == 0:
            return

        # Accumulation z-max par cellule (elevation map persistante)
        flat = iy * self.NX + ix
        elev_flat = self.elevation.reshape(-1)
        order = np.argsort(flat)
        flat_s, z_s = flat[order], z[order]
        uniq, first = np.unique(flat_s, return_index=True)
        zmax = np.maximum.reduceat(z_s, first)
        cur = elev_flat[uniq]
        elev_flat[uniq] = np.where(np.isnan(cur), zmax, np.maximum(cur, zmax))

        # Classification + publications
        cost = self._classify()
        stamp = self.get_clock().now().to_msg()
        self._publish_costmap(cost, stamp)
        self._publish_obstacles(cost, stamp)
        self._publish_gridmap(cost, stamp)

    @staticmethod
    def _fill_nan(E, passes=8):
        """Bouche les cellules inconnues (occlusion) par moyenne des voisins,
        pour pouvoir calculer une pente continue. Ne sert qu'au calcul interne;
        le masque `known` d'origine reste utilise pour l'affichage."""
        F = E.copy()
        for _ in range(passes):
            m = np.isnan(F)
            if not m.any():
                break
            nb = np.stack([np.roll(F, 1, 0), np.roll(F, -1, 0),
                           np.roll(F, 1, 1), np.roll(F, -1, 1)])
            with np.errstate(invalid='ignore'):
                nb_mean = np.nanmean(nb, axis=0)
            F = np.where(m & ~np.isnan(nb_mean), nb_mean, F)
        return np.nan_to_num(F, nan=0.0)

    # ── Traversabilite : pente sur fenetre robot -> cout 0-100 ────────
    def _classify(self):
        E = self.elevation
        known = ~np.isnan(E)

        # Pente globale sur une fenetre ~ empattement (difference centrale).
        # Repartit le denivele sur la distance horizontale : un escalier
        # regulier lit comme une rampe, pas comme une suite de falaises.
        W = max(1, int(round(SLOPE_WINDOW / RESOLUTION)))
        Ef = self._fill_nan(E)
        gx = (np.roll(Ef, -W, 1) - np.roll(Ef, W, 1)) / (2 * W * RESOLUTION)
        gy = (np.roll(Ef, -W, 0) - np.roll(Ef, W, 0)) / (2 * W * RESOLUTION)
        slope_deg = np.degrees(np.arctan(np.hypot(gx, gy)))

        # Cout gradue en deux segments :
        #  - SLOPE_FLAT..SLOPE_CLIMB : plateau bas-cout (0..COST_AT_CLIMB), terrain
        #    franchissable par le go2w (escaliers) -> faiblement penalise pour que
        #    MPPI/planner accepte de passer dessus plutot que de contourner.
        #  - SLOPE_CLIMB..SLOPE_MAX  : montee rapide (COST_AT_CLIMB..99) vers le letal.
        cost = np.zeros_like(E)
        seg1 = np.clip(
            (slope_deg - SLOPE_FLAT_DEG) / (SLOPE_CLIMB_DEG - SLOPE_FLAT_DEG),
            0.0, 1.0) * COST_AT_CLIMB
        seg2 = COST_AT_CLIMB + np.clip(
            (slope_deg - SLOPE_CLIMB_DEG) / (SLOPE_MAX_DEG - SLOPE_CLIMB_DEG),
            0.0, 1.0) * (99 - COST_AT_CLIMB)
        cost = np.where(slope_deg <= SLOPE_CLIMB_DEG, seg1, seg2)
        cost = np.where(slope_deg >= SLOPE_MAX_DEG, 100, cost)

        # Garde-fou : vraie falaise verticale (mur) -> letal, quelle que soit
        # la pente lissee. max |dz| vers les 4 voisins immediats.
        step = np.zeros_like(E)
        for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            shifted = np.full_like(E, np.nan)
            sj = slice(max(0, dj), self.NY + min(0, dj))
            si = slice(max(0, di), self.NX + min(0, di))
            tj = slice(max(0, -dj), self.NY + min(0, -dj))
            ti = slice(max(0, -di), self.NX + min(0, -di))
            shifted[tj, ti] = E[sj, si]
            dz = np.abs(E - shifted)
            valid = known & ~np.isnan(shifted)
            step = np.where(valid & (dz > step), dz, step)
        cost = np.where(step >= ABS_STEP_LETHAL, 100, cost)

        cost = cost.astype(np.int16)
        cost[~known] = -1          # inconnu
        return cost

    # ── 1. OccupancyGrid gradue ───────────────────────────────────────
    def _publish_costmap(self, cost, stamp):
        grid = OccupancyGrid()
        grid.header = Header(stamp=stamp, frame_id='odom')
        grid.info.resolution = RESOLUTION
        grid.info.width = self.NX
        grid.info.height = self.NY
        grid.info.origin.position.x = ORIGIN_X
        grid.info.origin.position.y = ORIGIN_Y
        grid.info.origin.orientation.w = 1.0
        grid.data = cost.astype(np.int8).reshape(-1).tolist()
        self.costmap_pub.publish(grid)

    # ── 2. Nuage des cellules letales ─────────────────────────────────
    def _publish_obstacles(self, cost, stamp):
        jj, ii = np.where(cost >= 100)
        if len(ii) == 0:
            # publie un nuage vide (laisse l'ObstacleLayer raytracer/clear)
            pts = np.zeros((0, 3), dtype=np.float32)
        else:
            x = ORIGIN_X + (ii + 0.5) * RESOLUTION
            y = ORIGIN_Y + (jj + 0.5) * RESOLUTION
            z = np.nan_to_num(self.elevation[jj, ii], nan=0.0)
            pts = np.stack([x, y, z], axis=1).astype(np.float32)

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg = PointCloud2(
            header=Header(stamp=stamp, frame_id='odom'),
            height=1, width=len(pts), is_dense=True, is_bigendian=False,
            fields=fields, point_step=12, row_step=12 * len(pts),
            data=pts.tobytes())
        self.obstacle_pub.publish(msg)

    # ── 3. grid_map_msgs/GridMap (elevation + traversabilite) ─────────
    def _publish_gridmap(self, cost, stamp):
        gm = GridMap()
        gm.header = Header(stamp=stamp, frame_id='odom')
        gm.info.resolution = RESOLUTION
        gm.info.length_x = SIZE_X
        gm.info.length_y = SIZE_Y
        gm.info.pose.position.x = ORIGIN_X + SIZE_X / 2.0
        gm.info.pose.position.y = ORIGIN_Y + SIZE_Y / 2.0
        gm.info.pose.orientation.w = 1.0
        gm.layers = ['elevation', 'traversability']
        gm.basic_layers = ['elevation']

        trav = cost.astype(np.float32)
        trav[cost < 0] = np.nan
        gm.data = [self._to_multiarray(self.elevation),
                   self._to_multiarray(trav)]
        gm.outer_start_index = 0
        gm.inner_start_index = 0
        self.gridmap_pub.publish(gm)

    def _to_multiarray(self, arr_jy_ix):
        """Convertit notre grille [j(y), i(x)] vers le layout grid_map.

        grid_map : matrice (nRows=NX en x, nCols=NY en y), index (0,0) au
        coin x-max/y-max, stockage column-major -> data[i + j*nRows].
        """
        # x descendant sur les lignes, y descendant sur les colonnes
        M = arr_jy_ix[::-1, ::-1].T            # -> shape (NX, NY)
        out = Float32MultiArray()
        out.layout.dim = [
            MultiArrayDimension(label='column_index', size=self.NY,
                                stride=self.NY * self.NX),
            MultiArrayDimension(label='row_index', size=self.NX,
                                stride=self.NX),
        ]
        out.data = M.flatten(order='F').tolist()   # column-major
        return out


def main():
    rclpy.init()
    node = TraversabilityMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
