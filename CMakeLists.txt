cmake_minimum_required(VERSION 3.8)
project(motion_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencias ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

# ── Ejecutable ────────────────────────────────────────────────────────────────
add_executable(velocity_controller_node
  src/velocity_command_computer.cpp
  src/proportional_velocity_controller_node.cpp
  src/main.cpp
)

target_include_directories(velocity_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(velocity_controller_node PUBLIC cxx_std_17)

ament_target_dependencies(velocity_controller_node
  rclcpp
  geometry_msgs
  robot_interfaces
)

install(TARGETS velocity_controller_node
  DESTINATION lib/${PROJECT_NAME})

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
