#!/bin/bash
set -e

# Entrypoint script for Jetson ROS2 Docker Container
echo "=========================================="
echo "Starting Jetson ROS2 Humble Container"
echo "=========================================="

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source /opt/ros_ws/install/setup.bash 2>/dev/null || true

# Source workspace if it exists
if [ -f /workspace/install/setup.bash ]; then
    echo "Sourcing workspace at /workspace"
    source /workspace/install/setup.bash
fi

if [ -f /root/ros2_ws/install/setup.bash ]; then
    echo "Sourcing workspace at /root/ros2_ws"
    source /root/ros2_ws/install/setup.bash
fi

# Setup X11 permissions for display
if [ ! -z "$DISPLAY" ]; then
    echo "Setting up X11 display: $DISPLAY"
    xhost +local:docker 2>/dev/null || true
fi

# Verify CUDA installation
if command -v nvcc &> /dev/null; then
    echo "CUDA Version:"
    nvcc --version | grep "release"
fi

# Check for RealSense cameras
echo ""
echo "Checking for RealSense devices..."
if command -v rs-enumerate-devices &> /dev/null; then
    rs-enumerate-devices 2>/dev/null || echo "No RealSense devices found (or not yet connected)"
else
    echo "RealSense tools available. Connect cameras and use 'rs-enumerate-devices' to list them."
fi

# List available video devices
echo ""
echo "Available video devices:"
ls -la /dev/video* 2>/dev/null || echo "No video devices found"

# Setup udev (if running with proper privileges)
if [ -w /run/udev ]; then
    echo ""
    echo "Triggering udev rules..."
    udevadm control --reload-rules 2>/dev/null || true
    udevadm trigger 2>/dev/null || true
fi

echo ""
echo "=========================================="
echo "Container ready!"
echo "=========================================="
echo ""
echo "Useful commands:"
echo "  - List RealSense devices: rs-enumerate-devices"
echo "  - Test RealSense viewer: realsense-viewer"
echo "  - List ROS2 nodes: ros2 node list"
echo "  - Launch RealSense: ros2 launch realsense2_camera rs_launch.py"
echo "  - Build workspace: colcon build --symlink-install"
echo ""

# Execute the command passed to docker run
exec "$@"