cmake_minimum_required(VERSION 3.8)
project(amr_tb3_lidar_driver VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME}_core
  src/amr_tb3_lidar_driver/laser_scan_assembler.cpp
  src/amr_tb3_lidar_driver/lidar_parser.cpp
  src/amr_tb3_lidar_driver/serial_transport.cpp
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  sensor_msgs
)

add_executable(${PROJECT_NAME}_node
  src/amr_tb3_lidar_driver/lidar_driver_node.cpp
  src/main.cpp
)
target_include_directories(${PROJECT_NAME}_node
  PRIVATE
    include
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_node
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
