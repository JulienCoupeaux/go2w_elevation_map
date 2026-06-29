# go2w_elevation_map

2.5D **traversability / elevation-map** perception + **Nav2 MPPI** navigation stack
for the Unitree **GO2-W**, plus the bridges to run it on the **real robot**
(locomotion handled by the robot's onboard Unitree policy — this stack only
outputs `cmd_vel`).

> Simulation packages are intentionally **not** included here. This repo is the
> elevation-map / navigation stack meant to run on the real robot (or on top of
> any source publishing a 3D `/pointcloud`).

## Packages (`ros2_ws/src/`)

| Package | Role |
|---------|------|
| **go2w_perception** | `traversability_mapper`: `/pointcloud` → 2.5D elevation map → traversability costmap. Cost = terrain **slope over a robot-footprint window** (climbable stairs stay cheap, true walls/cliffs lethal). Publishes `/traversability/costmap` (OccupancyGrid), `/traversability/obstacles` (PointCloud2), `/traversability/grid_map` (GridMap). Also `twist_to_stamped`, `terrain_cost_publisher`. |
| **go2w_navigation** | Nav2 config + launch: **MPPI controller** on the 2.5D costmap (DiffDrive), NavFn planner, costmaps (Static + Obstacle + Inflation layers) in the `odom` frame. |
| **go2w_robot_bridge** | Real-robot interface. `sportmode_to_odom`: `/sportmodestate` → `/odom` + TF. `cmd_vel_to_sport`: `/cmd_vel_nav` → Unitree `Move` on `/api/sport/request` (with velocity clamping + STOPMOVE watchdog). `robot_bringup.launch.py` ties it all together. |
| **go2w_bringup** | RViz config (`go2w_mppi.rviz`). |

See **[docs/REAL_ROBOT_BRINGUP.md](docs/REAL_ROBOT_BRINGUP.md)** for the full
end-to-end plan, bring-up sequence, incremental test plan, and open questions.

## How traversability cost is defined

Per cell, from the accumulated elevation map, in `go2w_perception/traversability_mapper.py`:

1. **Slope** = terrain gradient over a `±SLOPE_WINDOW` window (≈ robot wheelbase) —
   robust to single risers and occlusion, so a regular staircase reads as a ramp.
2. **Cost mapping** (tunable constants):
   - `< SLOPE_FLAT_DEG` → 0
   - `SLOPE_FLAT_DEG .. SLOPE_CLIMB_DEG` → 0 .. `COST_AT_CLIMB` (cheap, climbable)
   - `SLOPE_CLIMB_DEG .. SLOPE_MAX_DEG` → ramps to lethal
   - `≥ SLOPE_MAX_DEG`, or a vertical step `≥ ABS_STEP_LETHAL` → **lethal (100)**

Re-tune `SLOPE_*` / `ABS_STEP_LETHAL` to the GO2-W's real climbing limits.

## Dependencies (not vendored here)

- ROS 2 **Humble**
- `nav2` (navigation2, nav2_mppi_controller), `grid_map` + `grid_map_rviz_plugin`
- **`unitree_ros2`** (provides `unitree_go` / `unitree_api` messages used by
  `go2w_robot_bridge`, and the CycloneDDS bridge to the robot)

## Build & run

```bash
# in your ROS 2 Humble workspace, with unitree_ros2 also present in src/
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Real robot (after sourcing unitree_ros2 + CYCLONEDDS_URI on the wired NIC,
# LiDAR driver publishing /pointcloud, robot standing/ready):
ros2 launch go2w_robot_bridge robot_bringup.launch.py use_rviz:=true
# then set a goal with the RViz "Nav2 Goal" tool.
```
