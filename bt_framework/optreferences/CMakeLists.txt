cmake_minimum_required(VERSION 3.8)
project(optreferences)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/TransitionToPose.action"
  "action/FollowBlueRov.action"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Library
## optreferences_action_server
add_library(optreferences_action_server
  src/optreferences_action_server.cpp
)
ament_target_dependencies(optreferences_action_server
  angles rclcpp rclcpp_action std_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
)
add_dependencies(optreferences_action_server ${PROJECT_NAME}__cpp)
target_link_libraries(optreferences_action_server ${cpp_typesupport_target})

## bb_position_hold_action_server
add_library(bb_position_hold_action_server
  src/bb_position_hold_action_server.cpp
)
ament_target_dependencies(bb_position_hold_action_server
  angles rclcpp rclcpp_action std_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs geometry_msgs nav2_util
)
add_dependencies(bb_position_hold_action_server ${PROJECT_NAME}__cpp)
target_link_libraries(bb_position_hold_action_server ${cpp_typesupport_target})

# executable
## optreferences_action_server_node
add_executable(optreferences_action_server_node
  src/optreferences_action_server_main.cpp
)
ament_target_dependencies(optreferences_action_server_node
  rclcpp rclcpp_action std_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
)
target_link_libraries(optreferences_action_server_node 
  optreferences_action_server
  ${cpp_typesupport_target}
)
add_dependencies(optreferences_action_server_node ${PROJECT_NAME}__cpp)

## bb_position_hold_action_server_node
add_executable(bb_position_hold_action_server_node
  src/bb_position_hold_action_server_main.cpp
)
ament_target_dependencies(bb_position_hold_action_server_node
  angles rclcpp rclcpp_action std_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs geometry_msgs nav2_util
)
target_link_libraries(bb_position_hold_action_server_node 
  bb_position_hold_action_server
  ${cpp_typesupport_target}
)
add_dependencies(bb_position_hold_action_server_node ${PROJECT_NAME}__cpp)



# install
install(TARGETS
  optreferences_action_server
  optreferences_action_server_node
  bb_position_hold_action_server
  bb_position_hold_action_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)


ament_package()