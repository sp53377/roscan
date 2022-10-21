cmake_minimum_required(VERSION 3.5)
project(canplay_pkg)

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

find_package(ament_cmake REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED)
find_package(can_interfaces REQUIRED)

add_executable(canplay_node
  src/canplay_node.cpp
  src/CanNode.cpp
  src/CanPlayer.cpp
  src/ProcessingFns.cpp
)
ament_target_dependencies(canplay_node rclcpp std_msgs can_interfaces)

target_include_directories(canplay_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>)

target_link_libraries(canplay_node Threads::Threads librt.so)

install(TARGETS canplay_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
