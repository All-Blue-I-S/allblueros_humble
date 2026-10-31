cmake_minimum_required(VERSION 3.8)
project(auv_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)

# Dependencies for custom interfaces
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# ============================================================
# Custom interfaces
# ============================================================

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Corner.msg"
  "action/Config.action"
  DEPENDENCIES
    action_msgs
    sensor_msgs
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
    src/identifica_cor.py
    src/yolo_detector.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  FILES
  src/ffc_rs_26_det_only.pt
  src/yolo26n.pt
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
