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
    target_frame = LaunchConfiguration('target_frame', default='base_link')
    scan_topics = LaunchConfiguration('scan_topics', default=["/sick_s30b/laser/scan0", "/sick_s30b/laser/scan1"])
    qos_profiles = LaunchConfiguration('qos_profiles', default=["reliable", "reliable"])
    transform_tolerance = LaunchConfiguration('transform_tolerance', default=0.1)
    rate = LaunchConfiguration('rate', default=30.0)
    queue_size = LaunchConfiguration('queue_size', default=10)
    max_range = LaunchConfiguration('max_range', default=30.0)
    min_range = LaunchConfiguration('min_range', default=0.06)
    max_angle = LaunchConfiguration('max_angle', default=3.141592654)
    min_angle = LaunchConfiguration('min_angle', default=-3.141592654)
    scan_time = LaunchConfiguration('scan_time', default=0.01)
    angle_increment = LaunchConfiguration('angle_increment', default=0.02)
    inf_epsilon = LaunchConfiguration('inf_epsilon', default=1.0)
    use_inf = LaunchConfiguration('use_inf', default=True)

    output_pointcloud_topic = LaunchConfiguration('output_pointcloud_topic', default="pointcloud")
    output_scan_topic = LaunchConfiguration('output_scan_topic', default="scan")

    return LaunchDescription([
        Node(
            package='laser_merger2',
            executable='laser_merger2',
            name='laser_merger2',
            output='screen',
            parameters=[{'target_frame': target_frame},
                        {'scan_topics': scan_topics},
                        {'qos_profiles': qos_profiles},
                        {'transform_tolerance': transform_tolerance},
                        {'rate': rate},
                        {'queue_size': queue_size},
                        {'max_range': max_range},
                        {'min_range': min_range},
                        {'max_angle': max_angle},
                        {'min_angle': min_angle},
                        {'scan_time': scan_time},
                        {'angle_increment': angle_increment},
                        {'inf_epsilon': inf_epsilon},
                        {'use_inf': use_inf}
            ],
            remappings=[
                ('/pointcloud', output_pointcloud_topic),
                ('/scan', output_scan_topic)
            ]
        ),
    ])
