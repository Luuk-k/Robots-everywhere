cmake_minimum_required(VERSION 3.10)
project(ForceSubscriberPlugin)

# Find Gazebo Harmonic (Ignition Sim)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-plugin2 REQUIRED)

# Include directories
include_directories(
    ${gz-sim8_INCLUDE_DIRS}
    ${gz-transport13_INCLUDE_DIRS}
    ${gz-math7_INCLUDE_DIRS}
    ${gz-plugin2_INCLUDE_DIRS}
)

# Compiler settings
add_library(ForceSubscriberPlugin SHARED ForceSubscriberPlugin.cc)

target_link_libraries(ForceSubscriberPlugin
    ${gz-sim8_LIBRARIES}
    ${gz-transport13_LIBRARIES}
    ${gz-math7_LIBRARIES}
    ${gz-plugin2_LIBRARIES}
)

# Install the plugin
install(TARGETS ForceSubscriberPlugin
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

