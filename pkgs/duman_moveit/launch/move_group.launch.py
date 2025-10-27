import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml 

def load_yaml(package_name, file_path):
    """Load yaml file from a package share directory."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as f:
        return yaml.safe_load(f)
    
def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("duman_arm", package_name="duman_moveit")
        .robot_description(file_path="config/duman_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/duman_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

#     # Start the actual move_group node/action server
#     run_move_group_node = Node(
#     package="moveit_ros_move_group",
#     executable="move_group",
#     output="screen",
#     parameters=[
#         moveit_config.to_dict(),
#         {"planning_plugin": ["ompl_interface/OMPLPlanner"]},
#     ],
# )   
     # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    moveit_pkg = get_package_share_directory("duman_moveit")
    ompl_planning_yaml = load_yaml("duman_moveit","config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            # sensors_3d,
            # robot_description_planning,
            ompl_planning_pipeline_config,
            moveit_config.trajectory_execution,
            # moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": False},
        ],
    )

    # # Servo node for realtime control
    servo_yaml = load_yaml("duman_moveit", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {"use_sim_time": False},
        ],
        output="screen",
    )
    # RViz
    rviz_config_file = (
        get_package_share_directory("duman_moveit") + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("duman_moveit"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "duman_left_controller",
        "duman_right_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            # servo_node
            # motion_planner
        ]
        + load_controllers
    )

# ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
# 