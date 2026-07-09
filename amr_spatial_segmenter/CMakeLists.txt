cmake_minimum_required(VERSION 3.8)
project(amr_spatial_segmenter VERSION 0.19.4 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME}_core
  src/${PROJECT_NAME}/connected_components.cpp
  src/${PROJECT_NAME}/distance_map.cpp
  src/${PROJECT_NAME}/occupancy_grid_view.cpp
  src/${PROJECT_NAME}/segment_classifier.cpp
  src/${PROJECT_NAME}/spatial_segmenter.cpp
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core
  amr_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
)

add_executable(${PROJECT_NAME}
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
)

install(TARGETS
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_spatial_segmentation
    test/test_spatial_segmentation.cpp
  )
  if(TARGET test_spatial_segmentation)
    target_link_libraries(test_spatial_segmentation ${PROJECT_NAME}_core)
    target_include_directories(test_spatial_segmentation PRIVATE include)
    ament_target_dependencies(test_spatial_segmentation geometry_msgs nav_msgs)
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  amr_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
)

ament_package()
