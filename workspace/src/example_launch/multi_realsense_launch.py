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
