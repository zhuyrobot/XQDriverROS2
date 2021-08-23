from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xqdriver_ros2',
            namespace='xqserial_server',
            executable='xqserial_server_node',
            name='xqserial_server_node_launch',
            output='screen'
         ),
    ])
