cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)


find_package(rclcpp REQUIRED)
find_package(rviz_visual_tools REQUIRED)  
# Executable: robot_state_and_model
add_executable(robot_state_and_model src/robot_state_and_model.cpp)
target_include_directories(robot_state_and_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_state_and_model PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(robot_state_and_model
  moveit_ros_planning_interface
  rclcpp
)

# Executable: psm
add_executable(psm src/psm.cpp)
target_include_directories(psm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(psm PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(psm
  moveit_ros_planning_interface
  rviz_visual_tools 
  rclcpp
)

# Executable: motion_planning
add_executable(motion_planning src/motion_planning.cpp)
target_include_directories(motion_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motion_planning PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(motion_planning
  moveit_ros_planning_interface
  rviz_visual_tools 
  rclcpp
  graph_msgs
  pluginlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
)

add_executable(forward_kinem src/forward_kinem.cpp)
target_include_directories(forward_kinem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(forward_kinem PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(forward_kinem
  moveit_ros_planning_interface
  rclcpp
)

add_executable(joint_goal src/joint_goal.cpp)
target_include_directories(joint_goal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joint_goal PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(joint_goal
  moveit_ros_planning_interface
  moveit_msgs
  rclcpp

)

add_executable(pose_goal src/pose_goal.cpp)
target_include_directories(pose_goal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pose_goal PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(pose_goal
  moveit_ros_planning_interface
  moveit_msgs
  rclcpp

)

add_executable(servo src/servo.cpp)
target_include_directories(servo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(servo PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(servo
  geometry_msgs
  std_msgs
  control_msgs
  rclcpp

)

# Install
install(TARGETS
  robot_state_and_model
  psm
  motion_planning
  forward_kinem
  joint_goal
  pose_goal
  servo
  DESTINATION lib/${PROJECT_NAME})

ament_package()
