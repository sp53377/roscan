cmake_minimum_required(VERSION 3.8)
project(roscan_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(can_interfaces REQUIRED)

add_executable(roscan_node 
	src/CanBridge.cpp
	src/roscan_node.cpp
	)
ament_target_dependencies(roscan_node rclcpp std_msgs can_interfaces)

target_include_directories(roscan_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>)
  
target_compile_features(roscan_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(roscan_node Threads::Threads librt.so)
install(TARGETS roscan_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
