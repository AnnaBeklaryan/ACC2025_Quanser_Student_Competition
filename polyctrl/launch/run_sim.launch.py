import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration)

from launch_ros.actions import Node

def generate_launch_description():

    use_sim = LaunchConfiguration('use_sim')

    use_sim_la = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Start robot in Gazebo simulation')

    rviz_config_dir = os.path.join(
        get_package_share_directory('polyctrl'),
        'rviz',
        'rviz_config.rviz')

    qcar2_cartographer_virtual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('qcar2_nodes'),
                'launch',
                'qcar2_cartographer_virtual_launch.py')
        ])
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim}],
        output='screen'
    )

    # Make sure use_sim_la is declared first
    return LaunchDescription([
        use_sim_la,
        qcar2_cartographer_virtual_launch,
        rviz2
    ])
