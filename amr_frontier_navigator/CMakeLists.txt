cmake_minimum_required(VERSION 3.8)
project(amr_frontier_navigator VERSION 0.19.4 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_core
  src/${PROJECT_NAME}/frontier_utils.cpp
  src/${PROJECT_NAME}/frontier_navigator.cpp
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core
  amr_msgs
  geometry_msgs
  lifecycle_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(ft_navigator
  src/main.cpp
)

target_link_libraries(ft_navigator
  ${PROJECT_NAME}_core
)

install(TARGETS
  ${PROJECT_NAME}_core
  ft_navigator
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_frontier_utils
    test/test_frontier_utils.cpp
  )
  if(TARGET test_frontier_utils)
    target_link_libraries(test_frontier_utils ${PROJECT_NAME}_core)
    target_include_directories(test_frontier_utils PRIVATE include)
    ament_target_dependencies(test_frontier_utils geometry_msgs nav_msgs)
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  amr_msgs
  geometry_msgs
  lifecycle_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_package()
