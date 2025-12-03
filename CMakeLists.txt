cmake_minimum_required(VERSION 3.8)
project(ap1_control)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ap1_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
)

# ------------------------------------------------------------------------------
# EXECUTABLE: control_node
# Merge both versions: include your controllers + upstream structure
# ------------------------------------------------------------------------------

add_executable(control_node
  src/main.cpp
  src/control_node.cpp
  src/ackermann_controller.cpp
  src/pd_controller.cpp
)

target_include_directories(control_node PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(
  control_node
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
  ap1_msgs
)

target_link_libraries(control_node
  yaml-cpp
)

install(TARGETS control_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

ament_package()
