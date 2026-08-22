cmake_minimum_required(VERSION 3.8)

project(bt_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)

# Python
find_package(rclpy REQUIRED)

# ROS 2 C++
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# Mensagens padrão
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# TF2
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# MAVROS
find_package(mavros_msgs REQUIRED)

# Localização do pacote
find_package(ament_index_cpp REQUIRED)

# BehaviorTree.CPP
find_package(behaviortree_cpp_v3 REQUIRED)

# Geração de interfaces .msg / .srv / .action
find_package(rosidl_default_generators REQUIRED)

# Outros pacotes ROS
find_package(auv_navigation REQUIRED)

# ============================================================
# Custom interfaces
# ============================================================

rosidl_generate_interfaces(${PROJECT_NAME}

  # ----------------------------------------------------------
  # Services
  # ----------------------------------------------------------

  "srv/CheckEmpty.srv"
  "srv/GetVisionData.srv"

  # ----------------------------------------------------------
  # Actions
  # ----------------------------------------------------------

  "action/Align.action"
  "action/Detect.action"
  "action/Go2.action"

  # ----------------------------------------------------------
  # Dependências utilizadas pelas interfaces
  # ----------------------------------------------------------

  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
)


rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

# ============================================================
# C++ nodes
# ============================================================

add_executable(bt_executor_mission

  src/bt_executor_mission.cpp

  src/go2_node.cpp
  src/align_node.cpp
  src/rotate_node.cpp

  src/ActivateClaw_node.cpp
  src/DropMarkers_node.cpp

  src/fire_node.cpp
  src/FirstItem_node.cpp
  src/Increment_node.cpp

  src/Save_Pose.cpp

  src/detect_node.cpp
  src/emerge_node.cpp

  src/check_empty_table_node.cpp
  src/colect_data_node.cpp
)


ament_target_dependencies(
  bt_executor_mission

  rclcpp
  rclcpp_action

  std_msgs
  geometry_msgs
  nav_msgs

  tf2
  tf2_ros
  tf2_geometry_msgs

  mavros_msgs

  ament_index_cpp

  auv_navigation

  behaviortree_cpp_v3
)

target_link_libraries(bt_executor_mission
  "${cpp_typesupport_target}"
)


# ------------------------------------------------------------
# Dependência das interfaces geradas
# ------------------------------------------------------------

# add_dependencies(
#   bt_executor_mission
#   ${PROJECT_NAME}__rosidl_generator_cpp
# )

# ------------------------------------------------------------
# Instalação
# ------------------------------------------------------------

install(
  TARGETS
    bt_executor_mission

  DESTINATION lib/${PROJECT_NAME}
)


# ============================================================
# Planner
# ============================================================

add_executable(mission_planner_node
  src/planner.cpp
)


ament_target_dependencies(
  mission_planner_node

  rclcpp
  rclcpp_action

  geometry_msgs
  nav_msgs

  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(mission_planner_node
  "${cpp_typesupport_target}"
)

# add_dependencies(
#   mission_planner_node
#   ${PROJECT_NAME}__rosidl_generator_cpp
# )


install(
  TARGETS
    mission_planner_node

  DESTINATION lib/${PROJECT_NAME}
)


# ============================================================
# Include directory
# ============================================================

target_include_directories(bt_executor_mission
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(mission_planner_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)


# ============================================================
# Python nodes
# ============================================================

install(
  PROGRAMS
    scripts/Security_node.py

  DESTINATION lib/${PROJECT_NAME}
)


# ============================================================
# Behavior Tree XML
# ============================================================

install(
  DIRECTORY
    trees/

  DESTINATION
    share/${PROJECT_NAME}/trees
)


# ============================================================
# Tests
# ============================================================

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

endif()


# ============================================================
# Package
# ============================================================

ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()
