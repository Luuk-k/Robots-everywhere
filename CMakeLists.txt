cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(RobotClient)

# Find the Gazebo msgs library
find_package(gz-msgs10 QUIET REQUIRED)
find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

include_directories(${CMAKE_BINARY_DIR})

add_executable(${PROJECT_NAME} ForceTest.cc)
target_link_libraries(${PROJECT_NAME} ${GZ-MSGS_LIBRARIES})
target_link_libraries(${PROJECT_NAME} gz-transport${GZ_TRANSPORT_VER}::core)
