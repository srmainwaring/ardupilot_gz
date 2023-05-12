cmake_minimum_required(VERSION 3.5)
project(ardupilot_gz_application)

# --------------------------------------------------------------------------- #
# Find dependencies.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)

# --------------------------------------------------------------------------- #
# Build tests

if(BUILD_TESTING)
  # Override default flake8 configuration.
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_SOURCE_DIR}/.flake8)

  # Add linters.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# --------------------------------------------------------------------------- #
#  Install.

# Install Python package.
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

# Install Python executables / scripts.
install(PROGRAMS
  scripts/odometry_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------------------------------- #
# build tests

if(BUILD_TESTING)
  # Override default flake8 configuration.
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_SOURCE_DIR}/.flake8)

  # Add linters.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Add python tests.
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    # Add tests here, for example:
    # tests/test_a.py
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()
endif()

# --------------------------------------------------------------------------- #
# Call last.

ament_package()
