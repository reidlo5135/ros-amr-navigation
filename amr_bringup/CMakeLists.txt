cmake_minimum_required(VERSION 3.8)
project(amr_bringup VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_schema
  src/amr_bringup/robot_bringup.cpp
)
target_include_directories(${PROJECT_NAME}_schema
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_base
  src/amr_bringup/base/base_driver_node.cpp
  src/amr_bringup/base/differential_drive_odometry.cpp
  src/amr_bringup/base/opencr_protocol.cpp
  src/amr_bringup/base/opencr_sdk_wrapper.cpp
  src/amr_bringup/base/serial_transport.cpp
)
target_include_directories(${PROJECT_NAME}_base
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_base
  ${PROJECT_NAME}_schema
)
ament_target_dependencies(${PROJECT_NAME}_base
  dynamixel_sdk
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
)

add_library(${PROJECT_NAME}_lidar
  src/amr_bringup/lidar/lidar_driver_node.cpp
  src/amr_bringup/lidar/lidar_parser.cpp
  src/amr_bringup/lidar/laser_scan_assembler.cpp
  src/amr_bringup/lidar/serial_transport.cpp
)
target_include_directories(${PROJECT_NAME}_lidar
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_lidar
  ${PROJECT_NAME}_schema
)
ament_target_dependencies(${PROJECT_NAME}_lidar
  rclcpp
  sensor_msgs
)

add_executable(${PROJECT_NAME}_hardware
  src/main.cpp
)
target_include_directories(${PROJECT_NAME}_hardware
  PRIVATE
    include
)
target_link_libraries(${PROJECT_NAME}_hardware
  ${PROJECT_NAME}_base
  ${PROJECT_NAME}_lidar
  ${PROJECT_NAME}_schema
)
ament_target_dependencies(${PROJECT_NAME}_hardware
  rclcpp
)

install(TARGETS
  ${PROJECT_NAME}_schema
  ${PROJECT_NAME}_base
  ${PROJECT_NAME}_lidar
  ${PROJECT_NAME}_hardware
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  script/send_goal.sh
  script/initialpose.sh
  DESTINATION share/${PROJECT_NAME}/script
)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
