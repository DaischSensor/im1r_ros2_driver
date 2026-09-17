from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    frame_id = LaunchConfiguration('frame_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='IM1R',
        ),
        Node(
            package='im1r_ros2_driver',
            executable='im1r_driver_node',
            name='im1r_driver_node',
            output='screen',
            parameters=[
                {'serial_port': serial_port},
                {'baud_rate': baud_rate},
                {'frame_id': frame_id},
            ],
        ),
    ])
