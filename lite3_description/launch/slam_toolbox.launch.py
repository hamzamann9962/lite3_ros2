import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('lite3_description'), 'config', 'slam_toolbox', 'slam.yaml')

    slam = Node(package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[config_dir])

    ls_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/lslidar.launch.py']),
        launch_arguments={'rviz': 'true'}.items()
    )

    return LaunchDescription([
        slam,
        ls_driver
    ])
