cmake_minimum_required(VERSION 3.8)
project(duman_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    urdf
    rviz
    models
    sdf
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

# set(SCRIPTS_DIR scripts)
# install(PROGRAMS
#     # ${SCRIPTS_DIR}/nav2_api.py
#     DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
