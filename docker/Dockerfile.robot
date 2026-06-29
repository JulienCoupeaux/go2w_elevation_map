# ─────────────────────────────────────────────────────────────────────
# go2w elevation/nav stack — image LEGERE pour le VRAI robot (ARM64 / Jetson)
#
# CPU-only : Nav2 + MPPI + grid_map + nos packages. PAS de CUDA (le CUDA ne
# servait qu'a la simulation MuJoCo, qu'on ne lance pas sur le robot).
# `ros:humble` est multi-arch -> se build NATIVEMENT sur la Jetson ARM en
# quelques minutes (pas d'emulation, pas d'image x86 a transferer).
#
# Build (sur l'ordi ARM du robot, depuis la racine du repo) :
#   docker build -f docker/Dockerfile.robot -t go2w-nav:arm .
#
# Le code (nos packages + unitree_ros2) est bind-monte au run, pas baked,
# pour iterer sans rebuild d'image. Voir docker/README_ROBOT.md.
# ─────────────────────────────────────────────────────────────────────
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-nav2-mppi-controller \
      ros-humble-grid-map \
      ros-humble-grid-map-msgs \
      ros-humble-grid-map-rviz-plugin \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-tf2-ros \
      ros-humble-tf2-tools \
      python3-colcon-common-extensions \
      python3-numpy \
      git \
    && rm -rf /var/lib/apt/lists/*

# Unitree communique en CycloneDDS -> on l'impose pour voir /utlidar/cloud,
# /sportmodestate et publier /api/sport/request.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS automatiquement dans chaque shell interactif.
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /ros2_ws
CMD ["bash"]
