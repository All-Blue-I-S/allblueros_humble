cmake_minimum_required(VERSION 3.8)
project(auv_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)

find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# ============================================================
# Messages
# ============================================================

rosidl_generate_interfaces(${PROJECT_NAME}

  "msg/CurveReference.msg"

  DEPENDENCIES
    geometry_msgs
    std_msgs
)

# ============================================================
# Python nodes
# ============================================================

install(
  PROGRAMS
    scripts/controller_mux.py
    scripts/curve_reference.py
    scripts/dvz_controller.py
    scripts/dvz_mux.py
    scripts/dvz_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    scripts/curve3d.py
    scripts/dvz.py
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

ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()
