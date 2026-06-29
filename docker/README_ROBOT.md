# Running the go2w elevation/nav stack on the REAL GO2-W

This guide is for deploying **Julien's 2.5D traversability + Nav2 MPPI navigation
stack** on the physical GO2-W. Follow it top to bottom.

---

## 0. What this stack does (1 minute read)

It turns the robot's 3D LiDAR into a **2.5D traversability costmap** and runs a
**Nav2 MPPI** planner/controller on it. It does **not** do locomotion — it only
outputs a velocity command that we forward to the robot's **onboard Unitree
policy** (the robot walks/climbs itself).

```
/utlidar/cloud ─► traversability_mapper ─► 2.5D costmap ─► Nav2 (MPPI)
                                                              │
/sportmodestate ─► sportmode_to_odom ─► /odom + TF           ▼
                                              cmd_vel ─► cmd_vel_to_sport
                                                  ─► /api/sport/request ─► robot
Goal: RViz "Nav2 Goal"  ─►  /navigate_to_pose
```

Cost of a cell = local terrain **slope** (over a robot-footprint window), capped
at the GO2-W's real limits (climbable slope 35°, step/drop 70 cm). Flat = free,
steep/high = lethal. Thresholds are **live-tunable ROS params** (see §6).

---

## 1. Architecture: what runs where

- **Robot's native system (ROS 2 Foxy):** already publishes `/utlidar/cloud`
  (LiDAR, `sensor_msgs/PointCloud2`, frame `utlidar_lidar`) and `/sportmodestate`
  (`unitree_go/SportModeState`). You don't touch this — it just needs to be up.
- **Our stack (ROS 2 Humble):** runs in a **container** on the robot's ARM
  compute. It subscribes to those topics over DDS, runs Nav2, and publishes
  `/api/sport/request` back to the robot.

> The robot compute is **ARM64**. A x86 image will not run on it — that's why we
> build a small ARM image natively here (our stack is CPU-only; no CUDA/GPU).

---

## 2. Get the code

```bash
git clone https://github.com/JulienCoupeaux/go2w_elevation_map.git
cd go2w_elevation_map

# Our nodes need the Unitree message packages (unitree_go, unitree_api).
# Put unitree_ros2 in the workspace src (same .msg the robot uses, rebuilt for Humble):
cd ros2_ws/src
git clone https://github.com/koki67/unitree_ros2.git
cd ../..
```

---

## 3. Build the ARM image (native, on the robot, ~few minutes)

```bash
docker build -f docker/Dockerfile.robot -t go2w-nav:arm .
```
The image = ROS 2 Humble + Nav2 (incl. MPPI) + grid_map + RViz + CycloneDDS.
No code is baked in — the workspace is bind-mounted at run time so you can edit
& rebuild without rebuilding the image.

---

## 4. Run the container

```bash
docker run -it --rm \
   --network=host \
   -e ROS_DOMAIN_ID=<same value as the robot's Foxy> \
   -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
   -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
   -v $PWD/ros2_ws:/ros2_ws \
   go2w-nav:arm
```
- `--network=host` + matching `ROS_DOMAIN_ID` + CycloneDDS → the container sees
  the robot's topics. **These three must be right or nothing connects.**
- `DISPLAY` / X11 lines are only needed if you want RViz from the container;
  otherwise drop them and run RViz on a laptop on the same network.

---

## 5. Build the workspace + verify connectivity

```bash
# inside the container
cd /ros2_ws
colcon build --packages-up-to go2w_robot_bridge go2w_navigation go2w_bringup
source install/setup.bash
```
`--packages-up-to` builds only what's needed (our packages + the unitree
messages) and **never builds the simulation packages**.

**The make-or-break check** — does the container see the robot?
```bash
ros2 topic list | grep -E "utlidar/cloud|sportmodestate"
```
- Both appear → connected, continue.
- Missing → DDS issue: check `ROS_DOMAIN_ID`, `--network=host`, and that
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (the robot uses CycloneDDS).

---

## 6. Launch + drive

```bash
ros2 launch go2w_robot_bridge robot_bringup.launch.py
# add  use_rviz:=false  if you run RViz elsewhere
# add  pointcloud_topic:=/utlidar/cloud_base  to try the other cloud
```
This starts: `sportmode_to_odom`, TFs (`map→odom`, `base_link→utlidar_lidar`),
`traversability_mapper`, Nav2 (planner + MPPI + lifecycle), `cmd_vel_to_sport`,
and RViz.

**Before sending a goal:** put the robot **standing / ready to walk**
(BalanceStand). `cmd_vel_to_sport` does *not* auto-stand it.

**Send a goal:** RViz "Nav2 Goal" tool (flat ground first), or:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

---

## 7. Tuning the costmap live (no rebuild)

The traversability thresholds are ROS params on `/traversability_mapper`,
read every cycle. Defaults = GO2-W specs.

| Param | Default | Meaning |
|-------|---------|---------|
| `slope_max_deg` | 35.0 | slope → lethal above (deg) |
| `abs_step_lethal` | 0.70 | vertical step/drop → lethal above (m) |
| `slope_climb_deg` | 28.0 | low-cost knee |
| `slope_flat_deg` | 10.0 | flat → cost 0 |
| `cost_at_climb` | 25 | cost at the knee |
| `slope_window` | 0.30 | slope-averaging window (m) |

```bash
ros2 param set /traversability_mapper slope_max_deg 30.0      # on the fly
# or at launch:  ros2 run ... --ros-args -p slope_max_deg:=30.0
```
Rule of thumb: robot avoids something it could cross → raise `slope_max_deg` /
`abs_step_lethal`; robot heads into something impassable → lower them. You see
the effect live on the costmap in RViz.

---

## 8. Troubleshooting

| Symptom | Likely cause / fix |
|---------|--------------------|
| `colcon build` fails on `unitree_go`/`unitree_api` | `unitree_ros2` not in `ros2_ws/src` (step 2) |
| `ros2 topic list` doesn't show robot topics | DDS: wrong `ROS_DOMAIN_ID`, missing `--network=host`, or RMW not CycloneDDS |
| Costmap empty / no costmap | LiDAR topic name differs → `pointcloud_topic:=<real topic>`; check `ros2 topic hz /utlidar/cloud` |
| Cloud/costmap tilted or floating in RViz | LiDAR extrinsic — adjust the `base_link→utlidar_lidar` TF in `robot_bringup.launch.py` (currently 0.171, 0, 0.0908) |
| Robot doesn't move on a goal | It must be standing (BalanceStand) and in a walking sport mode; verify `Move` works first with a manual `ros2 topic pub /cmd_vel_nav ...` |
| Robot avoids climbable terrain | Loosen `slope_max_deg` / `abs_step_lethal` (§7) |

---

## 9. Key config reference

- **LiDAR topic:** `/utlidar/cloud` (override: `pointcloud_topic`)
- **LiDAR frame / extrinsic:** `utlidar_lidar`, `base_link→utlidar_lidar = (0.171, 0, 0.0908)`, identity rotation (XT-16 in the Go2 IMU frame; our `base_link` = IMU frame)
- **Localization:** `/sportmodestate → /odom` + TF, navigation in the `odom` frame (no global map; `map≡odom` identity)
- **Command:** `/cmd_vel_nav → Move(vx,vy,vyaw)` (api_id 1008) on `/api/sport/request`, clamped (vx 0.6 / vy 0.4 / wz 1.0) with a STOPMOVE watchdog
- **Packages:** `go2w_perception` (mapper), `go2w_navigation` (Nav2/MPPI), `go2w_robot_bridge` (odom + cmd_vel bridges + launch), `go2w_bringup` (RViz)
