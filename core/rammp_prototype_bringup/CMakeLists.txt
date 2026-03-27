cmake_minimum_required(VERSION 3.8)
project(rammp_prototype_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/rmw_config.dsv"
)

ament_package()