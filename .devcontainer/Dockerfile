# Use NVIDIA CUDA 12.1.1 on Ubuntu 22.04 as the base image for GPU support
FROM nvidia/cuda:12.1.1-devel-ubuntu22.04

# Set environment variables to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# Configure locale and install all dependencies in a single RUN to optimize build time and image size
RUN apt-get update && apt-get install -y \
    locales && locale-gen en_US.UTF-8 && \
    apt-get install -y --no-install-recommends \
    # Base development tools
    git cmake g++ build-essential curl software-properties-common \
    # Networking and utilities
    iproute2 iputils-ping vim python3-pip \    
    # Project-specific libraries for Unitree SDKs
    libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev && \
    rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble
# See official instructions: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
RUN apt-get update && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get install -y \
    # Install the full desktop version for GUI tools like RViz2
    ros-humble-desktop \
    # Install development tools like colcon
    ros-dev-tools \
    # Install CycloneDDS RMW implementation for ROS 2
    ros-humble-rmw-cyclonedds-cpp \
    # For unitree_ros2 dependency
    ros-humble-rosidl-generator-dds-idl \
    # Navigation : Nav2 (MPPI) + grid_map (carte 2.5D) + plugin RViz GridMap
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-grid-map \
    ros-humble-grid-map-rviz-plugin && \
    # Source ROS 2 setup file in bashrc to make it available in all new terminals
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    rm -rf /var/lib/apt/lists/*

# Set the working directory. The project files will be mounted here by VS Code.
WORKDIR /workspace

# The Unitree SDKs and ROS workspace are built automatically by the post-create.sh script
# when the container is created. See .devcontainer/post-create.sh for details.

# Command to keep the container running
CMD ["tail", "-f", "/dev/null"]
