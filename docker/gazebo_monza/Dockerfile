FROM ros:lyrical-ros-core-resolute

# Jetty は ROS Lyrical の vendor バイナリから導入する。
SHELL ["/bin/bash", "-eo", "pipefail", "-c"]
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ffmpeg libegl1 libgl1 libglvnd0 libglx0 python3-yaml \
        ros-lyrical-gz-sim-vendor ros-lyrical-gz-tools-vendor \
        ros-lyrical-ros-gz-bridge ros-lyrical-tf2-ros \
        ros-lyrical-ros2bag ros-lyrical-rosbag2-storage-mcap ros-lyrical-rosbag2-py ros-lyrical-tf2-msgs \
        ros-lyrical-rclpy ros-lyrical-sensor-msgs ros-lyrical-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && source /opt/ros/lyrical/setup.bash \
    && gz sim --help >/dev/null \
    && python3 -c 'from gz.sim import Model; from gz.math import Pose3d; import rclpy, rosbag2_py, yaml' \
    && ros2 bag record --help >/dev/null \
    && python3 -c 'from rosbag2_py import get_registered_readers, get_registered_writers; assert "mcap" in get_registered_readers() and "mcap" in get_registered_writers()' \
    && test -x /opt/ros/lyrical/lib/ros_gz_bridge/parameter_bridge \
    && ffmpeg -hide_banner -h encoder=libx264 2>&1 | grep 'Encoder libx264 '

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    FASTDDS_DEFAULT_PROFILES_FILE=/workspaces/atd/docker/middleware_profiles/fastdds.xml \
    ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT \
    GZ_IP=127.0.0.1
WORKDIR /workspaces/atd
