# Jetson Orin AGX ROS2 Docker Environment

## Quick Start

1. **Build the container:**
   ```bash
   docker-compose build
   ```

2. **Run the container:**
   ```bash
   ./run.sh
   # or
   docker-compose up -d
   ```

3. **Enter the container:**
   ```bash
   ./enter.sh
   # or
   docker exec -it jetson_ros2_perception bash
   ```

4. **Stop the container:**
   ```bash
   ./stop.sh
   # or
   docker-compose down
   ```

## Testing RealSense Cameras

Inside the container:

```bash
# List connected RealSense devices
rs-enumerate-devices

# Test with RealSense viewer
realsense-viewer

# Launch single camera with ROS2
ros2 launch realsense2_camera rs_launch.py

# Launch multiple cameras (after configuring serial numbers)
ros2 launch example_launch multi_realsense_launch.py
```

## Workspace Structure

- `workspace/`: The main ROS2 workspace
- `ros2_ws/`: Additional ROS2 workspace
- Both are automatically sourced in the container

## Building Your ROS2 Packages

```bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
```

## Common Issues

### No RealSense devices found
- Ensure USB devices are properly connected
- Check: `ls -la /dev/video*`
- Check: `lsusb | grep Intel`

### Display issues
- Run `xhost +local:docker` on host
- Verify DISPLAY variable: `echo $DISPLAY`

### Permission issues
- Container runs in privileged mode for full hardware access
- Ensure docker group membership: `groups`

## Features Included

- ROS2 Humble Desktop
- CUDA 12.6 (JetPack 6.1.2 compatible)
- Intel RealSense SDK 2.x
- RealSense ROS2 wrapper
- USB passthrough for multiple cameras
- Host networking for ROS2 DDS
- Display passthrough (X11)
- Persistent storage volumes
- GPU acceleration

## Advanced Configuration

Edit `docker-compose.yml` to customize:
- Memory limits
- Additional volumes
- Network settings
- Device mappings

## Useful Commands

```bash
# Check CUDA
nvcc --version

# Check ROS2
ros2 topic list

# Monitor topics
ros2 topic echo /camera/color/image_raw

# Visualize in RViz2
rviz2

# Build with specific packages
colcon build --packages-select my_package
```
