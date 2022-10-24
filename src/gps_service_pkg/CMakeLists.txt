cmake_minimum_required(VERSION 3.5)
project(gps_service_pkg)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(can_interfaces REQUIRED)

add_executable(gps_service_node 
	src/gps_service_node.cpp
	src/DirectionCoordinator.cpp
	src/GpsModel.cpp
	src/GpsService.cpp
	src/GpsServiceROS.cpp
	)

ament_target_dependencies(gps_service_node rclcpp std_msgs can_interfaces)

target_include_directories(gps_service_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>)
  
target_compile_features(gps_service_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(gps_service_node Threads::Threads librt.so)
install(TARGETS gps_service_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
