cmake_minimum_required(VERSION 3.8)
project(auv_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# ROS interfaces
find_package(rosidl_default_generators REQUIRED)

# ============================================================
# Custom interfaces
# ============================================================

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetTrackingMode.srv"
)

# ============================================================
# C++ nodes
# ============================================================

# Exemplo:
#
# add_executable(meu_no_cpp
#   src/meu_no_cpp.cpp
# )
#
# ament_target_dependencies(meu_no_cpp
#   rclcpp
#   geometry_msgs
# )
#
# install(
#   TARGETS
#     meu_no_cpp
#   DESTINATION lib/${PROJECT_NAME}
# )

# ============================================================
# Python nodes
# ============================================================

install(
  PROGRAMS
    scripts/thruster_allocator.py
    scripts/visual_servoing_controller.py
    scripts/pd_trajectory_controller.py
  DESTINATION lib/${PROJECT_NAME}
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

ament_export_dependencies(rosidl_default_runtime)

ament_package()
