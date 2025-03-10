cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(SensorCode)

# Find the Gazebo msgs library
find_package(gz-msgs11 QUIET REQUIRED)
find_package(gz-transport14 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport14_VERSION_MAJOR})

include_directories(${CMAKE_BINARY_DIR})

add_executable(${PROJECT_NAME} SensorCode.cc)
target_link_libraries(${PROJECT_NAME} ${GZ-MSGS_LIBRARIES})
target_link_libraries(${PROJECT_NAME} gz-transport${GZ_TRANSPORT_VER}::core)
