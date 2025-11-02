from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='duman_control',
            executable='move_duman_left.py',
            name='move_duman_left',
            output='screen'
        ),
        Node(
            package='duman_control',
            executable='move_duman_right.py',
            name='move_duman_right',
            output='screen'
        ),
        Node(
            package='duman_control',
            executable='passing.py',
            name='passing',
            output='screen'
        ),
        Node(
            package='duman_control',
            executable='ee_pose.py',
            name='ee_pose',
            output='screen'
        ),
        Node(
            package='duman_control',
            executable='duman_hardware.py',
            name='duman_hardware',
            output='screen'
        )
    ])