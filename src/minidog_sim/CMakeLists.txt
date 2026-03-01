cmake_minimum_required(VERSION 3.8)
project(minidog_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf config rviz worlds models webapp
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  minidog_sim/scan_filter.py
  minidog_sim/odom_stabilizer.py
  minidog_sim/data_relay.py
  minidog_sim/imu_fixup.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


