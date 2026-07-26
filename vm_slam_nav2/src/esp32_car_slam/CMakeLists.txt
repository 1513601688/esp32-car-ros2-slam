cmake_minimum_required(VERSION 3.8)
project(esp32_car_slam)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/save_map.py
    scripts/scan_normalizer.py
    scripts/cmd_vel_deadband_compensator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
