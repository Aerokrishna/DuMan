from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([
        ExecuteProcess(
            cmd=["/home/krishnapranav/ydlidar/linux_ros/ros2/run_ascamera_node.sh"],
            shell=True,
            output="screen"
        )
    ])
