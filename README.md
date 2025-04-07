# Simulation Code Project Robots Everywhere (0LAUK0)
Made by group 5  

Project done for the course Project Robots Everywhere (0LAUK0) for the TU/e.  
[Link](https://dsdwiki.wtb.tue.nl/wiki/PRE2024_3_Group5) to the wiki page.  

## How to run
This program is build to interact with the Gazebo simulation software.
The software is created to run on Ubuntu-24.04 and uses Gazebo harmonic (LTS).  

To install Gazebo click [here](https://gazebosim.org/docs/harmonic/getstarted/).  

To run the code a Gazebo GUI window needs to be open. This can be done using the following command.  
```
./sim.sh
```
After the GUI has opened the "Apply force and torque" plugin needs to be added. This plugin is used to add forces and torques to the robot.
The plugin can be added by clicking on the three dots at the top right of the GUI and then selecting the option called "Apply force and torque".  

The code can be run by performing the following commands in a seperate terminal:
```
./rebuild.sh
./build/RobotClient
```

After starting the code the client will wait for the start of the of the simulation, which can be started by clicking the play button in the left bottom of the Gazebo GUI.
The client code will then terminate after finding a crack.  

Pausing the simulation will pause movement, but does not guarantee that the robot will continue correctly after unpausing.  

The client code can also be stopped by focussing on the terminal and pressing CTRL+C.

## File structure
The simulation world is specified in the world.sdf file and can be opened by using the sim.sh script.  

The Robot code is written in one file named main.cc. This file contains all the code needed to run the robot client.  

CMakeList.txt constains the cmake build code. The code can be build with the rebuild.sh and the build.sh script. Rebuild will also rebuild the make files created by CMake.  
After building a build directory will be created. Here the executable is stored.  

The RobotModel and ship folder contain the robot and ship models.  

The data folder contains the possible data the robot could aquire. The file with the location of the crack can be found in the crack2 subfolder.  

Lastly, the old and ForceSubPlugin contain old code from previous attempts.

