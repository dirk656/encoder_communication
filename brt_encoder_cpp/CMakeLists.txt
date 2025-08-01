cmake_minimum_required(VERSION 3.8)
project(brt_encoder_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)

# Add library
add_library(can_interface src/can_interface.cpp)
target_include_directories(can_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(can_interface
  rclcpp
)

# Add executable
add_executable(brt_encoder_node src/brt_encoder_node.cpp)
target_link_libraries(brt_encoder_node
  can_interface
)
ament_target_dependencies(brt_encoder_node
  rclcpp
  std_msgs
  can_msgs
)

# Install
install(TARGETS
  can_interface
  brt_encoder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()