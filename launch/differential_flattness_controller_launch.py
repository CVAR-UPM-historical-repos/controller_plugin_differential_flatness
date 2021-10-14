from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

def generate_launch_description():
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