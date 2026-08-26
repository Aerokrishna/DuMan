from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    camera_launch = os.path.join(
        get_package_share_directory('duman_control'),
        'launch',
        'duman_camera.launch.py'
    )

    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(camera_launch)
        # ),
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
            executable='picknplace_left.py',
            name='picknplace_left',
            output='screen'
        ),
        Node(
            package='duman_control',
            executable='picknplace_right.py',
            name='picknplace_right',
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
            executable='duman_sim_client.py',
            name='duman_simulation',
            output='screen'
        ),
        Node(
            package='duman_control',
            executable='ex_collision_object.py',
            name='ex_collision_object',
            output='screen'
        ),
        # Node(
        #     package='duman_control',
        #     executable='dock_right.py',
        #     name='dock_right',
        #     output='screen'
        # ),
        # Node(
        #     package='duman_control',
        #     executable='dock_left.py',
        #     name='dock_left',
        #     output='screen'
        # )
    ])