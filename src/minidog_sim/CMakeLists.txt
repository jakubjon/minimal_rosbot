cmake_minimum_required(VERSION 3.8)
project(minidog_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf config rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


