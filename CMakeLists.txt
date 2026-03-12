cmake_minimum_required(VERSION 3.5)
project(im1r_ros2_driver)

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
find_package(im1r_ros2_interface REQUIRED)

include_directories(include)

# Create the driver node library
add_library(im1r_driver_lib
  src/im1r_driver.cpp
)
target_include_directories(im1r_driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(im1r_driver_lib
  rclcpp
  sensor_msgs
  im1r_ros2_interface
)

# Create the executable
add_executable(im1r_driver_node src/main.cpp)
target_link_libraries(im1r_driver_node im1r_driver_lib)
ament_target_dependencies(im1r_driver_node
  rclcpp
  sensor_msgs
  im1r_ros2_interface
)

# Install
install(TARGETS im1r_driver_lib im1r_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
