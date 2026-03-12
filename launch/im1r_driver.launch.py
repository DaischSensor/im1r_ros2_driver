from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='im1r_ros2_driver',
            executable='im1r_driver_node',
            name='im1r_driver_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'baud_rate': 115200},
                {'frame_id': 'IM1R'}
            ]
        )
    ])
