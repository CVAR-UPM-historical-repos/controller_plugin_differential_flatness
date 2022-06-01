import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('differential_flatness_based_controller'),
        'config',
        'default_controller.yaml'
        )
    node=Node(
            package="differential_flatness_based_controller",
            executable="differential_flatness_based_controller_node",
            name="differential_flatness_controller",
            namespace=LaunchConfiguration('drone_id'),
            parameters=[config],
            output="screen",
            emulate_tty=True,
        )    
    ld.add_action(DeclareLaunchArgument('drone_id', default_value='drone0'))
    ld.add_action(node)
    return ld
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        Node(
            package="differential_flatness_based_controller",
            executable="differential_flatness_based_controller_node",
            name="differential_flatness_controller",
            namespace=LaunchConfiguration('drone_id'),
            output="screen",
            emulate_tty=True,
        )
    ])
