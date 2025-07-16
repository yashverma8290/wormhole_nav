cmake_minimum_required(VERSION 3.8)
project(worhole_nav)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

## Add your action definition
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MultiMapNav.action"
  DEPENDENCIES action_msgs
)

include_directories(include)

add_executable(wormhole_navigator src/wormhole_navigator.cpp)

ament_target_dependencies(wormhole_navigator
  rclcpp
  rclcpp_action
  nav2_msgs
)

## 💡 Fix: Link against sqlite3 here
target_link_libraries(wormhole_navigator sqlite3)

## Link the generated action types
rosidl_target_interfaces(wormhole_navigator
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

## 📦 Install binaries
install(TARGETS wormhole_navigator
  DESTINATION lib/${PROJECT_NAME})

## 📂 Install important directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY urdf                     # ←✅ this line installs dummy_robot.urdf
  DESTINATION share/${PROJECT_NAME})

## 🔁 Install script
install(PROGRAMS switch_map.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()

