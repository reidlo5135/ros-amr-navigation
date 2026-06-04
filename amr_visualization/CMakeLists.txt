cmake_minimum_required(VERSION 3.8)
project(amr_visualization VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui OpenGL OpenGLWidgets Widgets)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

set(${PROJECT_NAME}_SOURCES
  include/amr_visualization/main_window.hpp
  include/amr_visualization/operator_state.hpp
  include/amr_visualization/robot_open_gl_widget.hpp
  include/amr_visualization/robot_model_renderer.hpp
  include/amr_visualization/ros_worker.hpp
  include/amr_visualization/scene_widget.hpp
  resources/resources.qrc
  src/main.cpp
  src/amr_visualization/main_window.cpp
  src/amr_visualization/operator_state.cpp
  src/amr_visualization/robot_open_gl_widget.cpp
  src/amr_visualization/robot_model_renderer.cpp
  src/amr_visualization/ros_worker.cpp
  src/amr_visualization/scene_widget.cpp
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
  ament_index_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  tf2_msgs
)

target_link_libraries(${PROJECT_NAME}
  Qt6::Core
  Qt6::Gui
  Qt6::OpenGL
  Qt6::OpenGLWidgets
  Qt6::Widgets
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  assets/icons/rcs.svg
  DESTINATION share/icons/hicolor/scalable/apps
  RENAME ${PROJECT_NAME}.svg
)

install(FILES
  resources/${PROJECT_NAME}.desktop
  DESTINATION share/applications
)

install(PROGRAMS
  scripts/${PROJECT_NAME}
  DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
