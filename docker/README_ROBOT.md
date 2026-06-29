# Running on the real GO2-W (ARM64 / Jetson)

The robot's compute is **ARM64** (Jetson). The laptop's x86 CUDA dev image will
**not** run there. Instead, build a small **CPU-only** ARM image natively on the
robot — our stack (Nav2 + MPPI + grid_map + bridges) needs no GPU; CUDA was only
for the MuJoCo simulation, which we do not run on the robot.

## 1. Get the code on the robot

```bash
git clone https://github.com/JulienCoupeaux/go2w_elevation_map.git
cd go2w_elevation_map

# Our packages need the Unitree message packages (unitree_go, unitree_api).
# Put unitree_ros2 (or at least those two message packages) in the workspace src:
cd ros2_ws/src
git clone https://github.com/koki67/unitree_ros2.git    # or copy the team's copy
cd ../..
```

> The robot's native Foxy already publishes `unitree_go`/`unitree_api` types; we
> rebuild the **same .msg** definitions under Humble so DDS interop works.

## 2. Build the ARM image (native on the robot, a few minutes)

```bash
docker build -f docker/Dockerfile.robot -t go2w-nav:arm .
```

## 3. Run the container

```bash
docker run -it --rm \
   --network=host \
   -e ROS_DOMAIN_ID=<same domain as the robot's Foxy> \
   -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
   -v $PWD/ros2_ws:/ros2_ws \
   go2w-nav:arm
```
`--network=host` + matching `ROS_DOMAIN_ID` + CycloneDDS = the container sees the
robot's topics. Add `-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix` if you
want RViz from the container (otherwise run RViz on a laptop on the same network).

## 4. Build the workspace + verify it sees the robot

```bash
# inside the container
cd /ros2_ws
colcon build --packages-up-to go2w_robot_bridge go2w_navigation go2w_bringup
source install/setup.bash

# CRUCIAL: does the container see the robot's topics?
ros2 topic list | grep -E "utlidar/cloud|sportmodestate"
```
If those two topics show up → you're connected. If not, it's a DDS/domain/network
issue (check `ROS_DOMAIN_ID`, CycloneDDS interface, `--network=host`).

## 5. Launch

```bash
ros2 launch go2w_robot_bridge robot_bringup.launch.py
# robot standing (BalanceStand), then send a goal (RViz "Nav2 Goal" or CLI)
```

## Notes
- We build only `--packages-up-to go2w_robot_bridge go2w_navigation go2w_bringup`,
  so the **simulation packages are never built** (no MuJoCo/pinocchio needed).
- `pointcloud_topic` defaults to `/utlidar/cloud`; override at launch if needed.
- The lidar TF `base_link→utlidar_lidar = (0.171, 0, 0.0908)` is in the launch.
