cmake_minimum_required(VERSION 3.5)
project(ardupilot_gz_arm_moveit)

# --------------------------------------------------------------------------- #
# Find dependencies.
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# --------------------------------------------------------------------------- #
#  Install.
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY rviz/   DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY srdf/   DESTINATION share/${PROJECT_NAME}/srdf)
install(DIRECTORY urdf/   DESTINATION share/${PROJECT_NAME}/urdf)

install(PROGRAMS
  scripts/moveit_to_gz_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------------------------------- #
# Call last.
ament_package()
