cmake_minimum_required(VERSION 3.8)
project(bt_policy)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(action_tutorials_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(optreferences REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(vision_msgs REQUIRED)

# Generate messages/actions in CURRENT package
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Bt.action"
  "msg/BtManagerReply.msg"
  "srv/HelpRequest.srv"
  "srv/GetDetections.srv"
  # "msg/ClusterObjInfo.msg"
  # "msg/LabeledObjInfo.msg"
  DEPENDENCIES std_msgs geometry_msgs vision_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)


# target_link_libraries(utils ${PROJECT_NAME}_LIBRARIES)
# add_dependencies(utils ${PROJECT_NAME}__cpp)
# target_link_libraries(utils ${cpp_typesupport_target})

# UTILS
add_library(utils STATIC nodes/utils.cpp)
ament_target_dependencies(utils
  behaviortree_cpp
)

# EMERGENCY - TO BE REMOVED(?)
add_library(emergency STATIC 
    src/emergency.cpp)
ament_target_dependencies(emergency
  rclcpp 
  behaviortree_cpp
)

# Control nodes 
add_library(control_nodes STATIC 
    nodes/post_cond_control_nodes.cpp)
ament_target_dependencies(control_nodes
  behaviortree_cpp 
)

# HSN - Help Seeker Node
add_library(emergency_handling STATIC 
    nodes/help_seeker_node_base.cpp
    nodes/contextual_condition_base.cpp)
ament_target_dependencies(emergency_handling
  rclcpp 
  rclcpp_action
  behaviortree_cpp
  std_msgs
    )
target_link_libraries(emergency_handling
  utils
  emergency
  control_nodes
  ${cpp_typesupport_target}
)

# BT Manager
add_library(bt_manager STATIC
    nodes/bt_manager.cpp
)
ament_target_dependencies(bt_manager
    rclcpp
    behaviortree_cpp
    yaml-cpp
)
target_link_libraries(bt_manager
  ${cpp_typesupport_target}
  ${YAML_CPP_LIBRARIES}
)

# Nodes
add_library(nodes STATIC
    nodes/crossdoor_nodes.cpp
    nodes/dummy_nodes.cpp
    nodes/movebase_node.cpp
    nodes/test_help_seeker_node.cpp
    nodes/transition_to_pose_node.cpp
    nodes/follow_bluerov_node.cpp
    nodes/inspect_object_node.cpp
    nodes/deocclude_fish_node.cpp
    nodes/check_no_occluder_node.cpp
    nodes/release_tether_node.cpp
)

ament_target_dependencies(nodes
    action_tutorials_interfaces
    behaviortree_cpp
    optreferences
    rclcpp
    rclcpp_action
    std_msgs
    std_srvs
    vision_msgs
)

target_include_directories(nodes PRIVATE
    emergency_handling
    ${cpp_typesupport_target}
    include)

# BT Manager
# add_library(bt_manager STATIC 
#     nodes/bt_manager.cpp)
# ament_target_dependencies(bt_manager
#   rclcpp 
#   behaviortree_cpp
# )
# target_link_libraries(bt_manager
#   emergency
#   utils
#   nodes
#   control_nodes
#   # emergency_handling
#   ${cpp_typesupport_target}
# )

# Node utils
# DO NOT USE THIS
add_library(nodes_utils STATIC 
    nodes/nodes_utils.cpp)
ament_target_dependencies(nodes_utils
  rclcpp 
  behaviortree_cpp 
)
target_link_libraries(nodes_utils
  utils
  nodes
  control_nodes
  # emergency_handling
  ${cpp_typesupport_target}
)



# # Repeat same pattern for other libraries:
# # emergency, control_nodes, emergency_handling, nodes, nodes_utils, bt_manager
# # Ensure they all include ${PROJECT_NAME} in dependencies

# # Executables
function(compile_src name)
  add_executable(${name} src/${name}.cpp)
  ament_target_dependencies(${name}
    rclcpp
    behaviortree_cpp
  )
  target_link_libraries(${name}
    bt_manager
    nodes
    utils
    emergency
    nodes_utils
    control_nodes
    emergency_handling
    ${cpp_typesupport_target}
  )
  
  
endfunction()


compile_src("exec_bt_manager")

# compile_src("bt_manager_app")
# compile_src("bt_action_server")

# Installation
install(TARGETS 
  utils 
  # emergency control_nodes
  # bt_manager_app bt_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  bt_xml
  config
  # models
  DESTINATION share/${PROJECT_NAME}
)

# install(
#   PROGRAMS scripts/detect_object.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
