FROM ros:jazzy-ros-base

ENV ROS_VERSION=2
ENV ROS_DISTRO=jazzy
ENV ROS_ROOT=/opt/ros/$ROS_DISTRO
ENV RCUTILS_LOGGING_BUFFERED_STREAM=1
ENV RCUTILS_COLORIZED_OUTPUT=1
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1
ENV CMAKE_BUILD_TYPE=Release
ENV DEBIAN_FRONTEND=noninteractive

# --no-install-recommends
RUN echo '\
APT::Install-Recommends "0";\n\
APT::Install-Suggests "0";\n\
' > /etc/apt/apt.conf.d/01norecommend

# Install cyclonedds RMW
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN apt update && \
    apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Install foxglove bridge v0.8.5
RUN . $ROS_ROOT/setup.sh \
    && mkdir /tmp/foxglove-build \
    && cd /tmp/foxglove-build \
    && mkdir src \
    && echo "\
    - git:\n\
        local-name: foxglove-sdk/foxglove_bridge\n\
        uri: https://github.com/ros2-gbp/foxglove_bridge-release.git\n\
        version: release/$ROS_DISTRO/foxglove_bridge/0.8.5-1\n\
    " | vcs import src \
    && rosdep update \
    && apt update \
    && rosdep install --from-paths src --ignore-src -y \
    && colcon build --merge-install --install-base /opt/ros/$ROS_DISTRO \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf /tmp/* \
    && rm -rf /var/lib/apt/lists/*

# Install Nav2 dependencies
RUN apt update && \
    apt install -y \
    ros-$ROS_DISTRO-nav2-core \
    ros-$ROS_DISTRO-nav2-controller \
    ros-$ROS_DISTRO-nav2-bt-navigator \
    ros-$ROS_DISTRO-nav2-lifecycle-manager \
    ros-$ROS_DISTRO-nav2-behaviors \
    ros-$ROS_DISTRO-nav2-planner \
    ros-$ROS_DISTRO-nav2-navfn-planner \
    ros-$ROS_DISTRO-nav2-regulated-pure-pursuit-controller \
    ros-$ROS_DISTRO-nav2-loopback-sim \
    ros-$ROS_DISTRO-nav2-map-server \
    && rm -rf /var/lib/apt/lists/*

# Setup .bashrc
RUN echo '\
export LD_LIBRARY_PATH=$ROS_ROOT/lib/$(gcc -dumpmachine)\n\
source $ROS_ROOT/setup.bash\n\
LOCAL_SETUP="/src/install/setup.bash";\n\
if [ -f "$LOCAL_SETUP" ]; then source $LOCAL_SETUP; fi\n\
' >> /root/.bashrc

# Entrypoint
WORKDIR /src
ENTRYPOINT ["/bin/bash", "-lc"]
CMD ["trap : TERM INT; sleep infinity & wait"]
