cmake_minimum_required(VERSION 3.8)
project(worhole_nav)

## Core Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

## Action Definition
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MultiMapNav.action"
  DEPENDENCIES action_msgs
)

## Include headers
include_directories(include)

## Build the wormhole_navigator node
add_executable(wormhole_navigator src/wormhole_navigator.cpp)

ament_target_dependencies(wormhole_navigator
  rclcpp
  rclcpp_action
  nav2_msgs
)

## Link SQLite (required)
target_link_libraries(wormhole_navigator sqlite3)

## Link action interfaces
rosidl_target_interfaces(wormhole_navigator
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

## Install your node binary
install(TARGETS wormhole_navigator
  DESTINATION lib/${PROJECT_NAME}
)

## Install important directories
install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

## ✅ Install the maps directory (this was missing)
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

## Install helper script
install(PROGRAMS switch_map.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
