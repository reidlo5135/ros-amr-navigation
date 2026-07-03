cmake_minimum_required(VERSION 3.8)
project(amr_mcp_server VERSION 0.1.0 LANGUAGES CXX)

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
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CURL REQUIRED)

set(${PROJECT_NAME}_SOURCES
  src/amr_mcp_node.cpp
  src/ollama_client.cpp
  src/prompt_interpreter.cpp
  src/ros_tool_executor.cpp
  src/topic_snapshot.cpp
  src/main.cpp
)

add_executable(${PROJECT_NAME}
  ${${PROJECT_NAME}_SOURCES}
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    include
)

ament_target_dependencies(${PROJECT_NAME}
  amr_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
)

target_link_libraries(${PROJECT_NAME}
  CURL::libcurl
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
