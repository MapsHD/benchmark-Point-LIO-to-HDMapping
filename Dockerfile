ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}-ros-base
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    python3-pip \
    nlohmann-json3-dev \
    libpcl-dev \
    libgoogle-glog-dev \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-eigen-conversions \
    ros-${ROS_DISTRO}-rosbag
RUN pip3 install rosbags
RUN mkdir -p /test_ws/src
COPY src/ /test_ws/src

# Point-LIO submodule may be empty if not initialized on host.
# Clone it directly if the package.xml is missing.
RUN if [ ! -f /test_ws/src/Point-LIO/package.xml ]; then \
      rm -rf /test_ws/src/Point-LIO && \
      git clone --recurse-submodules https://github.com/hku-mars/Point-LIO.git /test_ws/src/Point-LIO; \
    fi

# Ensure ikd-Tree submodule is present inside Point-LIO
RUN if [ ! -f /test_ws/src/Point-LIO/include/ikd-Tree/ikd_Tree.cpp ]; then \
      mkdir -p /test_ws/src/Point-LIO/include && \
      rm -rf /test_ws/src/Point-LIO/include/ikd-Tree && \
      git clone --depth 1 https://github.com/hku-mars/ikd-Tree.git /test_ws/src/Point-LIO/include/ikd-Tree; \
    fi

# Ensure LASzip submodule is present for the converter
RUN if [ ! -f /test_ws/src/point-lio-to-hdmapping/src/3rdparty/LASzip/CMakeLists.txt ]; then \
      mkdir -p /test_ws/src/point-lio-to-hdmapping/src/3rdparty && \
      rm -rf /test_ws/src/point-lio-to-hdmapping/src/3rdparty/LASzip && \
      git clone --depth 1 https://github.com/LASzip/LASzip.git /test_ws/src/point-lio-to-hdmapping/src/3rdparty/LASzip; \
    fi

RUN if [ ! -f /test_ws/src/livox_ros_driver/package.xml ]; then rm -rf /test_ws/src/livox_ros_driver && git clone https://github.com/Livox-SDK/livox_ros_driver.git /test_ws/src/livox_ros_driver; fi
RUN cd /test_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make -DCMAKE_CXX_STANDARD=17
