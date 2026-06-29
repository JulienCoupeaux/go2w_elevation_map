import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

# ── Specs Go2W ────────────────────────────────────────────────────────
ROBOT_HEIGHT    = 0.50
MAX_STEP_HEIGHT = 0.70
MAX_SLOPE_DEG   = 35.0
MAX_SLOPE       = np.tan(np.radians(MAX_SLOPE_DEG))

# ── Voxel grid ────────────────────────────────────────────────────────
VOXEL_SIZE      = 0.10   # 10cm — moins de voxels, plus lisible
GRID_RADIUS     = 5.0    # seulement 5m autour du robot
PUBLISH_HZ      = 2.0

SLOPE_LOW_DEG   = 5.0
SLOPE_MED_DEG   = 15.0


class TerrainCostPublisher(Node):

    def __init__(self):
        super().__init__('terrain_cost_publisher')

        self.sub_ground = self.create_subscription(
            PointCloud2, '/rtabmap/local_grid_ground',
            self._on_ground, 10)
        self.sub_obstacles = self.create_subscription(
            PointCloud2, '/rtabmap/local_grid_obstacle',
            self._on_obstacles, 10)

        self.pub = self.create_publisher(MarkerArray, '/terrain_costmap_3d', 10)

        self._ground_msg = None
        self._obstacle_msg = None
        self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self.get_logger().info('TerrainCostPublisher pret - local grid mode')

    def _parse_pointcloud(self, msg):
        n = msg.width * msg.height
        if n == 0:
            return np.zeros((0, 3), dtype=np.float32)
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
        x = raw[:, 0:4].view(np.float32).flatten()
        y = raw[:, 4:8].view(np.float32).flatten()
        z = raw[:, 8:12].view(np.float32).flatten()
        xyz = np.stack([x, y, z], axis=1)
        return xyz[np.isfinite(xyz).all(axis=1)]

    def _on_ground(self, msg):
        self._ground_msg = msg

    def _on_obstacles(self, msg):
        self._obstacle_msg = msg

    def _classify_ground(self, pts):
        if len(pts) == 0:
            return {}
        r = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2)
        pts = pts[r < GRID_RADIUS]
        if len(pts) == 0:
            return {}

        ix = np.floor(pts[:, 0] / VOXEL_SIZE).astype(np.int32)
        iy = np.floor(pts[:, 1] / VOXEL_SIZE).astype(np.int32)

        cols = {}
        for i in range(len(pts)):
            key = (int(ix[i]), int(iy[i]))
            z = float(pts[i, 2])
            if key not in cols or z > cols[key]:
                cols[key] = z

        voxels = {}
        neighbors = [(-1,0,VOXEL_SIZE),(1,0,VOXEL_SIZE),
                     (0,-1,VOXEL_SIZE),(0,1,VOXEL_SIZE)]

        for (cx, cy), z_here in cols.items():
            iz_ground = int(np.floor(z_here / VOXEL_SIZE))
            max_dz = 0.0
            for dx, dy, dist_xy in neighbors:
                nb = (cx+dx, cy+dy)
                if nb in cols:
                    dz = abs(z_here - cols[nb])
                    if dz > max_dz:
                        max_dz = dz

            slope = max_dz / VOXEL_SIZE

            if max_dz > MAX_STEP_HEIGHT or slope > MAX_SLOPE:
                cost = 100
            elif max_dz > 0.03:
                t = (max_dz - 0.03) / (MAX_STEP_HEIGHT - 0.03)
                cost = int(10 + t * 80)
            elif slope > np.tan(np.radians(SLOPE_MED_DEG)):
                cost = 60
            elif slope > np.tan(np.radians(SLOPE_LOW_DEG)):
                cost = 30
            else:
                cost = 0

            # Un seul voxel au sol — pas d'empilement
            voxels[(cx, cy, iz_ground)] = cost

        return voxels

    @staticmethod
    def _cost_to_color(cost):
        if cost == 0:
            return ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.6)
        elif cost <= 30:
            return ColorRGBA(r=0.5, g=0.8, b=0.0, a=0.7)
        elif cost <= 60:
            t = (cost - 30) / 30.0
            return ColorRGBA(r=0.8+0.2*t, g=0.8-0.6*t, b=0.0, a=0.8)
        elif cost < 100:
            return ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.8)
        else:
            return ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)

    def _publish(self):
        if self._ground_msg is None and self._obstacle_msg is None:
            return

        voxel_costs = {}

        if self._ground_msg is not None:
            pts = self._parse_pointcloud(self._ground_msg)
            voxel_costs.update(self._classify_ground(pts))

        # Obstacles écrasent le sol
        if self._obstacle_msg is not None:
            pts = self._parse_pointcloud(self._obstacle_msg)
            if len(pts) > 0:
                r = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2)
                pts = pts[r < GRID_RADIUS]
                ix = np.floor(pts[:, 0] / VOXEL_SIZE).astype(np.int32)
                iy = np.floor(pts[:, 1] / VOXEL_SIZE).astype(np.int32)
                iz = np.floor(pts[:, 2] / VOXEL_SIZE).astype(np.int32)
                for i in range(len(pts)):
                    voxel_costs[(int(ix[i]), int(iy[i]), int(iz[i]))] = 100

        if not voxel_costs:
            return

        stamp = self.get_clock().now().to_msg()
        frame = self._ground_msg.header.frame_id if self._ground_msg else 'base_link'

        markers = MarkerArray()
        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = frame
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = frame
        m.ns = 'terrain_cost'
        m.id = 1
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.scale.x = VOXEL_SIZE * 0.9
        m.scale.y = VOXEL_SIZE * 0.9
        m.scale.z = VOXEL_SIZE * 0.9
        m.pose.orientation.w = 1.0

        for (ix, iy, iz), cost in voxel_costs.items():
            p = Point()
            p.x = (ix + 0.5) * VOXEL_SIZE
            p.y = (iy + 0.5) * VOXEL_SIZE
            p.z = (iz + 0.5) * VOXEL_SIZE
            m.points.append(p)
            m.colors.append(self._cost_to_color(cost))

        markers.markers.append(m)
        self.pub.publish(markers)
        self.get_logger().info(
            f'Publie {len(voxel_costs)} voxels — '
            f'obstacles: {sum(1 for c in voxel_costs.values() if c==100)}'
        )


def main():
    rclpy.init()
    node = TerrainCostPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
