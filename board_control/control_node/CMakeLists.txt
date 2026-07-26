cmake_minimum_required(VERSION 3.8)
project(esp32_car_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# --- 服务接口生成 ---
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TimedVelocity.srv"
  DEPENDENCIES geometry_msgs
)

add_library(serial_protocol src/serial_protocol.cpp)
target_include_directories(serial_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(sensor_publisher src/sensor_publisher.cpp)
target_include_directories(sensor_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(sensor_publisher
  serial_protocol
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

add_executable(esp32_car_node src/esp32_car_node.cpp)

# 开发板本地建图遥控。该节点只发布 /cmd_vel，不访问 ESP32 串口。
add_executable(mapping_teleop_node src/mapping_teleop_node.cpp)

target_include_directories(esp32_car_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 链接标准依赖 + 生成的服务接口
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(esp32_car_node
  sensor_publisher
  serial_protocol
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  "${cpp_typesupport_target}"
)

target_link_libraries(mapping_teleop_node
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
)

install(TARGETS esp32_car_node mapping_teleop_node sensor_publisher serial_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_executable(serial_protocol_test test/serial_protocol_test.cpp)
  target_link_libraries(serial_protocol_test serial_protocol)
  add_test(NAME serial_protocol_test COMMAND serial_protocol_test)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
