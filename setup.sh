#!/bin/bash

# Setup script for Jetson Orin AGX Docker Environment
set -e

echo "=========================================="
echo "Jetson Orin AGX Docker Setup"
echo "=========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running on Jetson
if [ ! -f /etc/nv_tegra_release ]; then
    echo -e "${RED}Warning: This doesn't appear to be a Jetson device${NC}"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Create necessary directories
echo -e "${GREEN}Creating directory structure...${NC}"
mkdir -p workspace/src
mkdir -p ros2_ws/src

# Setup X11 authentication for display passthrough
echo -e "${GREEN}Setting up X11 authentication...${NC}"
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Set proper permissions
chmod 777 $XAUTH

# Add current user to docker group if not already
if ! groups | grep -q docker; then
    echo -e "${YELLOW}Adding current user to docker group...${NC}"
    sudo usermod -aG docker $USER
    echo -e "${YELLOW}You may need to log out and back in for group changes to take effect${NC}"
fi

# Enable NVIDIA runtime as default
echo -e "${GREEN}Configuring NVIDIA Docker runtime...${NC}"
if [ -f /etc/docker/daemon.json ]; then
    echo -e "${YELLOW}/etc/docker/daemon.json already exists, skipping...${NC}"
else
    sudo tee /etc/docker/daemon.json > /dev/null <<EOF
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia"
}
EOF
    sudo systemctl restart docker
    echo -e "${GREEN}Docker daemon restarted with NVIDIA runtime${NC}"
fi

# Make entrypoint executable
chmod +x entrypoint.sh

# Create a helper script for running the container
echo -e "${GREEN}Creating helper scripts...${NC}"
cat > run.sh <<'EOF'
#!/bin/bash
# Helper script to run the container

# Ensure X11 socket permissions
xhost +local:docker

# Export display
export DISPLAY=:0

# Run docker-compose
docker-compose up -d

# Show logs
echo "Container started. Viewing logs (Ctrl+C to exit)..."
docker-compose logs -f
EOF

chmod +x run.sh

cat > enter.sh <<'EOF'
#!/bin/bash
# Helper script to enter the running container
docker exec -it jetson_ros2_perception bash
EOF

chmod +x enter.sh

cat > stop.sh <<'EOF'
#!/bin/bash
# Helper script to stop the container
docker-compose down
EOF

chmod +x stop.sh

# Create example launch file
mkdir -p workspace/src/example_launch
cat > workspace/src/example_launch/multi_realsense_launch.py <<'EOF'
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import os

def generate_launch_description():
    """
    Launch multiple RealSense cameras
    Modify serial numbers to match your cameras
    """
    
    # Camera configurations - UPDATE THESE WITH YOUR CAMERA SERIAL NUMBERS
    cameras = [
        {'name': 'camera_1', 'serial': ''},  # Add your serial here
        {'name': 'camera_2', 'serial': ''},  # Add your serial here
        # Add more cameras as needed
    ]
    
    nodes = []
    
    for camera in cameras:
        if camera['serial']:  # Only launch if serial number is provided
            node = Node(
                package='realsense2_camera',
                executable='realsense2_camera_node',
                name=camera['name'],
                namespace=camera['name'],
                parameters=[{
                    'serial_no': camera['serial'],
                    'enable_color': True,
                    'enable_depth': True,
                    'enable_infra1': False,
                    'enable_infra2': False,
                    'depth_module.profile': '640x480x30',
                    'rgb_camera.profile': '640x480x30',
                    'align_depth.enable': True,
                }],
                output='screen',
            )
            nodes.append(node)
    
    return LaunchDescription(nodes)
EOF

# Create README
cat > README.md <<'EOF'
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

- `workspace/`: Your main ROS2 workspace
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

- ✅ ROS2 Humble Desktop
- ✅ CUDA 12.6 (JetPack 6.1.2 compatible)
- ✅ Intel RealSense SDK 2.x
- ✅ RealSense ROS2 wrapper
- ✅ Full perception stack (PCL, OpenCV, image_pipeline)
- ✅ USB passthrough for multiple cameras
- ✅ Host networking for ROS2 DDS
- ✅ Display passthrough (X11)
- ✅ Persistent storage volumes
- ✅ GPU acceleration

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
EOF

echo ""
echo -e "${GREEN}=========================================="
echo "Setup complete!"
echo -e "==========================================${NC}"
echo ""
echo "Next steps:"
echo "  1. Review and update camera serial numbers in workspace/src/example_launch/multi_realsense_launch.py"
echo "  2. Build the container: docker-compose build"
echo "  3. Run the container: ./run.sh"
echo "  4. Enter the container: ./enter.sh"
echo ""
echo "See README.md for detailed usage instructions"
echo ""