cmake_minimum_required(VERSION 3.5)
project(duman_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(pymoveit2)

# Install examples
set(EXAMPLES_DIR examples)
set(CONTROL control)
set(HARDWARE hardware)


install(PROGRAMS
    ${EXAMPLES_DIR}/ex_collision_object.py
    ${EXAMPLES_DIR}/ex_joint_goal.py
    ${EXAMPLES_DIR}/ex_pose_goal.py
    ${EXAMPLES_DIR}/ex_servo.py
    ${EXAMPLES_DIR}/duman_pose_goal.py
    ${EXAMPLES_DIR}/duman_pose_goal_left.py
    ${EXAMPLES_DIR}/duman_joint_goal.py

    ${CONTROL}/move_duman_left.py
    ${CONTROL}/move_duman_right.py
    ${CONTROL}/ee_pose.py


    ${CONTROL}/test_client.py

    ${HARDWARE}/duman_hardware.py
    ${CONTROL}/duman_teleop.py
    ${CONTROL}/fsm_node.py
    ${CONTROL}/passing.py




    DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY examples/assets
  DESTINATION lib/${PROJECT_NAME}
)

# Setup the project
ament_package()
