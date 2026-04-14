cmake_minimum_required(VERSION 3.8)
project(amr_mqtt_server VERSION 0.6.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_path(PAHO_MQTT_INCLUDE_DIR MQTTClient.h)
find_library(PAHO_MQTT3C_LIBRARY NAMES paho-mqtt3c)

if(NOT PAHO_MQTT_INCLUDE_DIR OR NOT PAHO_MQTT3C_LIBRARY)
  message(FATAL_ERROR
    "libpaho-mqtt-dev was not found. Install it with: sudo apt install -y libpaho-mqtt-dev")
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(${PROJECT_NAME}
  src/amr_mqtt_server/node.cpp
  src/main.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    include
    ${PAHO_MQTT_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  ${PAHO_MQTT3C_LIBRARY}
  m
)

ament_target_dependencies(${PROJECT_NAME}
  action_msgs
  amr_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  tf2_msgs
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
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
