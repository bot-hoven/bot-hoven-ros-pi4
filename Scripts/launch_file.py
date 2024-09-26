from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='package_name',
            executable='gpio_node',
            name='gpio_node'
        ),
        Node(
            package='package_name',
            executable='spi_node',
            name='spi_node'
        ),
        Node(
            package='package_name',
            executable='i2c_node',
            name='i2c_node'
        ),
    ])