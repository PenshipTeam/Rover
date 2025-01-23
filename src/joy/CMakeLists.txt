cmake_minimum_required(VERSION 3.5)
project(joy)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Add the executable
add_executable(rov_joy_controller src/joy.cpp)
target_include_directories(rov_joy_controller PRIVATE include)
ament_target_dependencies(rov_joy_controller rclcpp sensor_msgs geometry_msgs)

# Install the executable
install(TARGETS rov_joy_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install the header files
install(DIRECTORY include/
  DESTINATION include
)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()