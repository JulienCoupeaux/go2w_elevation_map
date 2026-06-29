#!/bin/bash

# This script is executed after the container is created.
set -e

# --- 0. Fix "dubious ownership" error for ALL directories ---
# The wildcard '*' tells Git to trust every directory inside this container,
# which is safe since the container environment is isolated.
git config --global --add safe.directory '*'

echo "--- Starting post-create command ---"

echo "--- 1. Building C++ SDK (unitree_sdk2) ---"
cd /workspace/unitree_sdk2
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
make -j$(nproc) && make install

echo "--- 2. Building Python SDK (unitree_sdk2_python) ---"
cd /workspace/unitree_sdk2_python
pip3 install -e .

echo "--- 3. Building ROS 2 packages (unitree_ros2) ---"
# Source ROS2 base environment
source /opt/ros/humble/setup.bash

# Set environment variables required for the ROS 2 packages to find the C++ SDK.
export CMAKE_PREFIX_PATH=/opt/unitree_robotics:$CMAKE_PREFIX_PATH

# Build all packages in src/, including unitree_ros2.
# --symlink-install allows Python/Launch file changes to take effect without rebuilding
cd /workspace/ros2_ws
colcon build --symlink-install || true

echo "--- 4. Persisting environment variables for new terminals ---"
# This block writes all necessary setup commands to .bashrc.
# It will be executed every time a new terminal is opened in the container.
# Check if the setup is already in .bashrc to prevent duplication on rebuilds
if ! grep -q "source /workspace/install/setup.bash" /root/.bashrc; then
    
    echo "Appending ROS 2 and Unitree configurations to .bashrc..."
    
    # We use a quoted heredoc (<<'EOF') to prevent variable expansion during the cat command.
    # This ensures that ${UNITREE_NETWORK_INTERFACE} is written literally to .bashrc,
    # so it evaluates dynamically based on the container env every time you open a terminal.
cat <<'EOF' >> /root/.bashrc

# --- ROS 2 & Unitree Setup ---
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash

# Unitree SDK Paths
export CMAKE_PREFIX_PATH=/opt/unitree_robotics:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:$LD_LIBRARY_PATH

# CycloneDDS & Network Configuration
# This uses the interface defined in devcontainer.json
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${UNITREE_NETWORK_INTERFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
EOF

fi

echo "--- Post-create command finished ---"