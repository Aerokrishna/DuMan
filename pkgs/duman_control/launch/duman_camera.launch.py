from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        ExecuteProcess(
            cmd=["/home/krishnapranav/ydlidar/linux_ros/ros2/run_ascamera_node.sh"],
            shell=True,
            output="screen"
        ),
        Node(
            package='duman_control',
            executable='duman_camera.py',
            name='duman_camera',
            output='screen'
        ),
    ])
