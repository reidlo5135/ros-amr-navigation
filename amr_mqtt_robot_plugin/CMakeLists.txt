cmake_minimum_required(VERSION 3.8)
project(amr_mqtt_robot_plugin VERSION 0.5.0 LANGUAGES C)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_path(PAHO_MQTT_INCLUDE_DIR MQTTClient.h)
find_library(PAHO_MQTT3C_LIBRARY NAMES paho-mqtt3c)

if(NOT PAHO_MQTT_INCLUDE_DIR OR NOT PAHO_MQTT3C_LIBRARY)
  message(FATAL_ERROR
    "libpaho-mqtt-dev was not found. Install it with: sudo apt install -y libpaho-mqtt-dev")
endif()

include_directories(
  include
  include/${PROJECT_NAME}
  ${PAHO_MQTT_INCLUDE_DIR}
)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(rcl_yaml_param_parser REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(SOURCES
  src/${PROJECT_NAME}/node.c
  src/main.c
)

set(DEPENDS
  geometry_msgs
  nav_msgs
  rcl
  rclc
  rcl_yaml_param_parser
  rcutils
  rmw
  rosidl_runtime_c
  sensor_msgs
  std_msgs
)

set(EXEC_NAME
  ${PROJECT_NAME}
)

add_executable(${EXEC_NAME}
  ${SOURCES}
)

target_link_libraries(${EXEC_NAME}
  ${PAHO_MQTT3C_LIBRARY}
  m
)

ament_target_dependencies(${EXEC_NAME}
  ${DEPENDS}
)

install(TARGETS
  ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
