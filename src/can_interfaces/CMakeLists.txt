cmake_minimum_required(VERSION 3.5)
project(can_interfaces)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RegisterMsg.srv"
  "msg/BearingSpeed.msg"
  "msg/DirectionSpeed.msg"
  "msg/HeadingMode.msg"
  "msg/ReceiverInfo.msg"
  "msg/SatellitesUsed.msg"
  "msg/TimeDate.msg"
  "msg/DifferentialStatus.msg"
  "msg/GpsStatus.msg"
  "msg/PitchAltitude.msg"
  "msg/RollYawRate.msg"
  "msg/TerrainCompensation.msg"
  "msg/VehiclePosition.msg"
)

ament_package()
