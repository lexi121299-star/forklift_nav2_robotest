FROM ros:foxy

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
# CycloneDDS is the default RMW: Foxy's FastRTPS intermittently crashes a random
# lifecycle node during the Nav2 configure/activate storm with
# "failed to send response" (seen on amcl, bt_navigator, recoveries_server).
# CycloneDDS does not exhibit this. FastRTPS is still installed as a one-line
# fallback: override with RMW_IMPLEMENTATION=rmw_fastrtps_cpp.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pytest \
    ros-foxy-gazebo-msgs \
    ros-foxy-gazebo-ros \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-robot-state-publisher \
    ros-foxy-rmw-cyclonedds-cpp \
    ros-foxy-rmw-fastrtps-cpp \
    ros-foxy-rviz2 \
    ros-foxy-tf2-ros \
    ros-foxy-turtlebot3-cartographer \
    ros-foxy-turtlebot3-gazebo \
    ros-foxy-turtlebot3-navigation2 \
    ros-foxy-xacro \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY docker/foxy/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
