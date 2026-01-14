# Use NVIDIA L4T base image for Jetson
FROM nvcr.io/nvidia/l4t-cuda:12.2.12-devel

# Environment Setup
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}
ENV ROS_PYTHON_VERSION=3

# System Dependencies
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    wget \
    git \
    vim \
    nano \
    build-essential \
    cmake \
    python3-pip \
    python3-dev \
    udev \
    usbutils \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libssl-dev \
    libudev-dev \
    libv4l-dev \
    v4l-utils \
    mesa-utils \
    x11-apps \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ROS2 Humble Installation
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-ros-base \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

# ROS2 Perception Packages
RUN apt-get update && apt-get install -y \
    ros-humble-vision-opencv \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-image-pipeline \
    ros-humble-cv-bridge \
    ros-humble-camera-info-manager \
    ros-humble-camera-calibration \
    ros-humble-camera-calibration-parsers \
    ros-humble-depth-image-proc \
    ros-humble-stereo-image-proc \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-point-cloud-transport \
    ros-humble-laser-geometry \
    ros-humble-tf2-sensor-msgs \
    ros-humble-tf2-geometry-msgs \
    ros-humble-rqt-image-view \
    ros-humble-rviz2 \
    ros-humble-foxglove-bridge \
    ros-humble-diagnostic-updater \
    ros-humble-cyclonedds \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Intel RealSense SDK 2.x (Build from source for ARM64/Jetson)
ENV REALSENSE_BASE=/opt
ENV REALSENSE_DIR=${REALSENSE_BASE}/librealsense

# Install build dependencies for RealSense
RUN apt-get update && apt-get install -y \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone and build librealsense with CUDA support
RUN git clone https://github.com/IntelRealSense/librealsense.git ${REALSENSE_DIR} \
    && cd ${REALSENSE_DIR} \
    && mkdir build && cd build \
    && cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=true \
        -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_WITH_CUDA=true \
        -DBUILD_PYTHON_BINDINGS:bool=true \
        -DPYTHON_EXECUTABLE=$(which python3) \
        .. \
    && make -j$(nproc) install \
    && rm -rf ${REALSENSE_DIR}/build

# Install udev rules for RealSense
RUN curl https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules \
    -o /etc/udev/rules.d/99-realsense-libusb.rules

# Update library cache
RUN ldconfig

# Build xacro from source (since ros-humble-xacro binary not available for ARM64)
WORKDIR /opt/ros_deps_ws
RUN mkdir -p src && cd src \
    && git clone https://github.com/ros/xacro.git -b ros2 \
    && cd /opt/ros_deps_ws \
    && . /opt/ros/humble/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# RealSense ROS2 Wrapper (built from source)
WORKDIR /opt/ros_ws
RUN mkdir -p src && cd src \
    && git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development \
    && cd /opt/ros_ws \
    && . /opt/ros/humble/setup.sh \
    && . /opt/ros_deps_ws/install/setup.sh \
    && rosdep install -i --from-path src --rosdistro humble --skip-keys=librealsense2 -y || true \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to realsense2_camera realsense2_camera_msgs realsense2_description

# Additional Python packages for perception
RUN pip3 install --no-cache-dir \
    numpy \
    opencv-python \
    opencv-contrib-python \
    scipy \
    scikit-learn \
    pyrealsense2 \
    transforms3d \
    open3d

# Setup workspace
RUN mkdir -p /workspace/src

# Entrypoint setup
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set working directory
WORKDIR /workspace

# Setup ROS environment in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /opt/ros_deps_ws/install/setup.bash" >> /root/.bashrc \
    && echo "source /opt/ros_ws/install/setup.bash" >> /root/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc \
    && echo "export DISPLAY=:0" >> /root/.bashrc \
    && echo "export PYTHONPATH=\$PYTHONPATH:/usr/local/lib" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]