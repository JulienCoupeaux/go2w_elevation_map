# GO2-W — Nav2 MPPI 2.5D on the real robot: end-to-end plan

Goal of the experiment: run our **Nav2 MPPI controller on a 2.5D traversability
costmap** on the physical GO2-W. We do **not** run a custom locomotion
controller — we rely on the robot's **onboard Unitree policy (nomad mode)**.
Our stack's job is: build the costmap, plan, and **output a velocity command**
(`cmd_vel`) that we forward to the robot as an Unitree `Move` command.

This note formulates the full chain so Monday is "plug & debug", not "design".

---

## 1. The three things the system needs (per the researcher)

> "the system to *set the goal position*, the *robot know where it is*, then
> finally *execute the navigation*."

| Need | In simulation (free) | On the real robot | Status |
|------|----------------------|-------------------|--------|
| **Set the goal** | RViz "Nav2 Goal" tool | Same, goal expressed in **`odom`** frame (no map at first) | reused |
| **Robot knows where it is** | ground-truth TF from `mujoco_node` | `/sportmodestate` → `/odom` + TF `odom→base_link` | **new node `sportmode_to_odom`** ✅ |
| **Execute navigation** | `cmd_vel` → `mujoco_node` | `cmd_vel` → Unitree `Move(vx,vy,vyaw)` on `/api/sport/request` | **new node `cmd_vel_to_sport`** ✅ |

Everything else (perception `traversability_mapper`, Nav2 planner + **MPPI**
controller, costmap config) is **unchanged** — costmaps already run in the
`odom` frame.

---

## 2. End-to-end data flow (real robot)

```
[robot onboard, Unitree DDS over wired link]
  /sportmodestate ──► sportmode_to_odom ──► /odom + TF odom→base_link
  LiDAR 3D driver ──► /pointcloud (+ static TF base_link→lidar)
                                   │
                                   ▼
              traversability_mapper ──► /traversability/costmap (+ /obstacles)
                                   │
        Nav2 planner + MPPI controller (global_frame = odom) ──► /cmd_vel_nav
                                   │
              cmd_vel_to_sport ──► Move(vx,vy,vyaw) ──► /api/sport/request ──► robot
                                   ▲
        Goal: RViz "Nav2 Goal"  ──► /navigate_to_pose  (frame map≡odom)
```

A static identity TF `map→odom` is published so Nav2 (whose BT navigator uses
`map`) is happy while we actually navigate in `odom`. **No global map / SLAM**
for the first run — simplest path to a working demo. Add rtabmap SLAM later if
we need drift-free long-range goals.

---

## 3. New nodes (already written, built, smoke-tested in sim)

Package: `go2w_robot_bridge`.

### `cmd_vel_to_sport`  (execute navigation)
- Subscribes `/cmd_vel_nav` (`geometry_msgs/Twist`, MPPI output).
- Publishes Unitree `Request` on `/api/sport/request`:
  `Move` = api_id **1008**, `parameter` = JSON `{"x":vx, "y":vy, "z":vyaw}`.
- **Safety**: re-clamps to `vx_max/vy_max/wz_max` (defaults 0.6 / 0.4 / 1.0 for
  the first run); **watchdog** sends `STOPMOVE` (api_id 1003) if no command for
  `cmd_timeout` (0.5 s) — covers Nav2 crash / goal cancel / soft e-stop.
- **Does NOT auto-stand the robot** — operator controls physical startup.
- Verified: Twist `{x:0.5, z:0.3}` → `api_id 1008, {"x":0.5,"y":0.0,"z":0.3}`;
  on stop → `api_id 1003`.

### `sportmode_to_odom`  (robot knows where it is)
- Subscribes `/sportmodestate` (`unitree_go/SportModeState`, BEST_EFFORT QoS).
- Publishes `/odom` (`nav_msgs/Odometry`) + TF `odom→base_link` from
  `position[3]`, `imu_state.quaternion` (Unitree `[w,x,y,z]` → ROS `[x,y,z,w]`),
  `velocity[3]`, `yaw_speed`.
- Verified: fake state pos `(1.5,0.5,0.35)` → matching `/odom` + TF.

---

## 4. Bring-up sequence (on the robot's network)

```bash
# 0. Network: source unitree_ros2, CYCLONEDDS_URI on the wired NIC (see repo README)
#    Confirm comms:  ros2 topic echo /sportmodestate   (should stream)

# 1. Start the LiDAR 3D driver so it publishes /pointcloud  (manufacturer driver)

# 2. Put the robot up and ready to walk (Unitree app / joystick, or a one-shot
#    BalanceStand request). The robot must be STANDING before any Move.

# 3. Launch the whole stack
ros2 launch go2w_robot_bridge robot_bringup.launch.py use_rviz:=true

# 4. Set a goal in RViz ("Nav2 Goal"), a couple of meters ahead, flat ground first.
```

`robot_bringup.launch.py` starts: `sportmode_to_odom`, static `map→odom`,
static `base_link→lidar` (extrinsic placeholder), `traversability_mapper`,
`cmd_vel_to_sport`, and Nav2 (controller/planner/behavior/bt_navigator +
lifecycle autostart), + RViz.

---

## 5. Monday incremental test plan (de-risk, in order)

1. **Comms**: `/sportmodestate` streams; `sportmode_to_odom` → `/odom` moves
   sanely when you push the robot by hand / joystick it 1 m.
2. **Command path (robot on a stand or open space, low speed)**: manually
   `ros2 topic pub /cmd_vel_nav ... {linear.x: 0.2}` → robot walks forward;
   stop publishing → robot stops (STOPMOVE). Validate direction signs.
3. **Perception**: `/pointcloud` arrives; `/traversability/costmap` looks
   correct in RViz (flat = low cost, real obstacles = lethal). Check the
   LiDAR→base_link extrinsic and the cloud frame (see risks).
4. **Full loop**: short Nav2 goal on flat ground; then a goal across the
   test terrain. Watch `/plan` + MPPI `/trajectories` in RViz.

---

## 6. Known integration points / risks to verify with the robot

- **LiDAR frame**: `traversability_mapper` currently assumes the cloud is in
  `base_link`. The real LiDAR publishes in its own frame. Fix either by
  remapping/transforming the cloud into `base_link`, or by generalizing the
  mapper's TF lookup to use `msg.header.frame_id` (small change — flag it).
- **LiDAR extrinsic**: the static `base_link→lidar` TF in the launch is a
  placeholder `[0,0,0.3]` (sim value). Set the real mounted pose.
- **Quaternion / velocity conventions**: assumed Unitree quat `[w,x,y,z]` and
  body-frame `velocity`. Verify on hardware (param `velocity_in_body`).
- **Robot mode**: `Move` only takes effect in the right sport/locomotion mode.
  Confirm the exact mode for go2w stair-capable walking ("nomad"), and whether
  a mode switch / `BalanceStand` is needed first.
- **Velocity limits & footprint**: first run capped at vx 0.6 m/s; MPPI
  `vx_max` (1.0) and `robot_radius` (0.35) may need tuning to the real robot.
- **No map → drift**: navigating in `odom` means goals drift over long range.
  Fine for short runs; add SLAM for longer experiments.
- **Traversability thresholds**: stair-climbing capability is encoded in
  `traversability_mapper.py` (`SLOPE_*`, `ABS_STEP_LETHAL`). Re-tune to the
  GO2-W's real climbing limits once we see it walk.

---

## 7. Open questions for the researcher

1. Which **3D LiDAR** is mounted, what **topic** does it publish, and what is
   the **mounting transform** to `base_link`?
2. What **sport/locomotion mode** must the go2w be in for stair-capable walking,
   and is a programmatic mode switch available (api_id) or done via the app?
3. Are the `/sportmodestate` `position`/`velocity` good enough as odometry, or
   do you prefer we fuse IMU / run SLAM from the start?
4. Goal source: RViz tool on the laptop, or a scripted waypoint sequence?
