cmake_minimum_required(VERSION 3.8)
project(so_arm_100_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(so_arm_100_moveit src/so_arm_100_moveit.cpp)
target_include_directories(so_arm_100_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(so_arm_100_moveit PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  so_arm_100_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(robot_model_and_robot_state_tutorial src/robot_model_and_robot_state_tutorial.cpp)
target_include_directories(robot_model_and_robot_state_tutorial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(robot_model_and_robot_state_tutorial PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  robot_model_and_robot_state_tutorial
  "moveit_ros_planning_interface"
  "rclcpp"
)


# Install executables
install(TARGETS
  so_arm_100_moveit
  robot_model_and_robot_state_tutorial
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install configuration files
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
