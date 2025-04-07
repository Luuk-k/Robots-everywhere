cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(RobotClient)

# Find the Gazebo msgs library
find_package(gz-msgs10 QUIET REQUIRED)
find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
find_package(gz-physics7 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

include_directories(${CMAKE_BINARY_DIR})

add_executable(${PROJECT_NAME} main.cc)
target_link_libraries(${PROJECT_NAME} ${GZ-MSGS_LIBRARIES})

target_link_libraries(${PROJECT_NAME} ${gz-physics7_LIBRARIES})
target_link_libraries(${PROJECT_NAME} gz-transport${GZ_TRANSPORT_VER}::core)
