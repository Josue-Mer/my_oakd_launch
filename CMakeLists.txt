cmake_minimum_required(VERSION 3.5)
project(my_oakd_launch)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY launch my_oakd_launch flag_data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
 