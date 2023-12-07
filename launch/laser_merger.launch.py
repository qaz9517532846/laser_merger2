import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='laser_merger2',
            executable='laser_merger2',
            name='laser_merger2',
            output='screen',
            parameters=[{'target_frame': 'base_link'},
                        {'laser_num': 2},
                        {'transform_tolerance': 0.1},
                        {'rate': 30.0},
                        {'queue_size': 10},
                        {'max_range': 30.0},
                        {'min_range': 0.06},
                        {'max_angle': 3.141592654},
                        {'min_angle': -3.141592654},
                        {'scan_time': 0.01},
                        {'angle_increment': 0.02},
                        {'inf_epsilon': 1.0},
                        {'use_inf': True}],
            remappings=[("scan_0", "/sick_s30b/laser/scan0"),
                        ("scan_1", "/sick_s30b/laser/scan1")],),
    ])
