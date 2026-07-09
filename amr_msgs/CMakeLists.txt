cmake_minimum_required(VERSION 3.8)
project(amr_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToPose.action"
  "action/NavigateToPoses.action"
  "action/NavigateToUnknownPose.action"
  "msg/FrontierNavigationStatus.msg"
  "msg/MotionCommand.msg"
  "msg/ObstacleReport.msg"
  "msg/MotionStatus.msg"
  "msg/LocalPlanStatus.msg"
  "msg/SpatialSegment.msg"
  "msg/SpatialSegmentArray.msg"
  "srv/ClearCostmap.srv"
  "srv/AiChat.srv"
  "srv/PlanLocalEscape.srv"
  "srv/PlanRecovery.srv"
  "srv/PlanSegment.srv"
  "srv/PlanRoute.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
