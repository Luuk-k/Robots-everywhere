cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find the Gazebo msgs library
find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

include_directories(${CMAKE_BINARY_DIR})

add_executable(SensorCode SensorCode.cc)
target_link_libraries(SensorCode gz-transport${GZ_TRANSPORT_VER}::core)

