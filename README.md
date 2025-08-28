my_cpp_py_pkg (turn into HIRO-HIL-SERL-Utils)
================================
This repository contains a set of libraries and utils for the HIRO HIL-SERL implementation that you can find at this link: [hil-serl-hiro](https://github.com/claudio-dg/hil-serl-hiro/tree/hiro_simulation).
The main goal of these libraries is that of permitting a proper data exchange between Gym's environment and its robot counterpart, both for simulation case (i.e. with Mujoco) and both for the real UR case.
Moreover, here you can find the scripts to control the robot by using an XBOX joypad


Table of contents
----------------------

<!-- * [Introduction](#introduction) -->
* [Dependencies and Setup](#dependencies-and-setup)
* [Project structure](#project-structure)
* [Software Components and code description](#software-components-and-code-description)
* [Joypad Commands](#joypad-commands)
* [Limitations and Possible Improvements](#limitations-and-possible-improvements)

<!--
## Introduction

The goal of this assignment is to integrate the architecture developed in the [first assignment](https://github.com/claudio-dg/assignment_1) with a robotic simulation., in particular we have to Add a robot to a simulation environment and Integrate (if needed, modify it) the architecture that we have developed in the first assignment to the given scenario in such a way that:
* The robot is spawned in the initial position x = -6.0, y = 11.0
* Builds the "semantic" map of the environment by detecting, without moving the base of the robot, all seven markers that are present around it, by calling the provided service node.
* Starts the patrolling algorithm by relying on autonomous navigation strategies (mapping/planning) and on the information collected and stored in the ontology during the previous step.
* When a room is reached, perform a complete scan of the room (by rotating the base or the camera).

The Map
====================================================================
The 3D environment representation of the 2D map of former assignment, which has been used for this project is shown here below:

<p><p align="center">
<img src="https://github.com/claudio-dg/assignment2/blob/main/media/GazeboMap.png?raw=true" width="500" />
<p>

	
The Robot
====================================================================

For this project I built my own robot composed of a base chassis and an additional arm on top that combines prismatic and rotational joints. It presents a laser scan sensor located on the chassis and a camera on the end effector of the arm: the dimensions and joints of the arm have been specifically designed for the robot to be capable of detecting the Aruco Markers with the camera without moving its base. As you can see here below, i therefore made an arm with 4 links, where:
* The first one (Red cylinder) has a **Revolute joint** that allows to rotate of 360° along **Z-axis**
* The second one (Blue cylinder) has again a **Revolute joint** but this time along **X-axis**, to allow rotations as in the second image
* The third one (Green cylinder) has a **Prismatic joint** that moves along **Z-axis**, to allow increasing/decreasing the lenght of the arm according to the distance of the marker to detect
* The last one (Black parallelepiped) has a **Revolute joint** that allows to rotate along **X-axis** and has the camera (white cube) mounted on it.
	
<p><p align="center">
<img src="https://github.com/claudio-dg/assignment2/blob/main/media/My_robot1.png?raw=true" width="400" />
<p>
	
<p><p align="center">
<img src="https://github.com/claudio-dg/assignment2/blob/main/media/My_robot2.png?raw=true" width="400" />
<p>

Please find the code to create this robot in the [urdf](https://github.com/claudio-dg/assignment2/tree/main/urdf) folder.
##  Dependencies and Setup

In order to run correctly the project of this repository, some important dependencies have to be taken into account, therefore please make sure to have the following packages already installed in your ```ros_workspace```:
- First of all you will need to have the Pkg of my first assignment that you can find in this link: [assignment_1](https://github.com/claudio-dg/assignment_1/tree/second_assignment_changes). Please make sure to clone this repository and to be in the correct branch that i created that is ```second_assignment_changes```, where you can find the codes slightly modified in order to adapt them to this part of the project
- Additionally you will of course need the packages listed in the Readme of [assignment_1](https://github.com/claudio-dg/assignment_1/tree/second_assignment_changes), that for sake of simplicity I briefly list again here: [arch_skeleton](https://github.com/buoncubi/arch_skeleton), [topological_map](https://github.com/buoncubi/topological_map), [aRMOR](https://github.com/EmaroLab/armor) and [SMACH](http://wiki.ros.org/smach) libraries.
- **MoveBase** package, of the ROS Navigation stack :
```bash
$ sudo apt-get install ros-<ros_distro>-navigation
```
- [explore-lite](https://github.com/CarmineD8/m-explore) package
- [planning](https://github.com/CarmineD8/planning) package to use path planning algorithms
- [aruco_ros](https://github.com/CarmineD8/aruco_ros) package for being able to work with aruco markers.



```bash
$ Roscore &
$ rosrun armor execute it.emarolab.armor.ARMORMainService

## Project structure

The Overall project is based on the ROS scheme that is shown in the following ```rqt_graph```:
<!--
<p align="center">
<img src="https://github.com/claudio-dg/assignment2/blob/main/media/final_rosgraph.png?raw=true" width="850" />
<p>
 -->

 **Table for code structure**

| Code Directory | Description |
| --- | --- |
| [config.state_bridge_params.yaml](https://github.com/claudio-dg/my_cpp_py_pkg/blob/master/config/state_bridge_params.yaml) | Parameter file to select what data to "bridge". (currently for Simulation only) |
| [launch.state_bridge.launch.py](https://github.com/claudio-dg/my_cpp_py_pkg/blob/master/launch/state_bridge.launch.py) | Launch File to start "bridging" information between Mujoco and gym |
| msg | Main code for HIL-SERL |
| scripts | Main code for HIL-SERL |
| scripts.ForceBridgeNode.py | Main code for HIL-SERL |
| scripts.StateBridgeNode.py| Main code for HIL-SERL |
| scripts.RealStateBridgeNode.py | Main code for HIL-SERL |
| scripts.UR_joystick_move.py | Code to collect input from joystick and send them to the Bridge or directly to the Robot |



 
## Software Components and code description
	

```marker_publish``` node 
====================================================================

	
	
```marker_server``` node  
====================================================================
	

	
	
```move_arm_server``` node  
====================================================================

 

## Joypad Commands
The following pictures show how to use the XBOX pad to move and/or rotate the UR end effector. In the scripts **(metterli in una sottocartella e linkarla, ma conviene farlo da qua o da vecow e ushare?)** you can adjust the following parameters to adjust the control to your need: increase the scale value to have faster movements, or decrease it to have slower and more accurate control of the robot
```bash
Inserire Code snippet da vecow qui
```

UR_joystick_move: only translational movement
====================================================================
<p align="center">
  <img src="https://github.com/claudio-dg/hil-serl-hiro/blob/hiro_simulation/docs/images/joystick.png" width="550">
</p>

XXX_Rotation: include end effector rotational movement
====================================================================
<p align="center">
  <img src="https://github.com/claudio-dg/hil-serl-hiro/blob/hiro_simulation/docs/images/joystick_2.png" width="550">
</p>
	
	
 ## Limitations and Possible Improvements
 
sidered reach only when its exact given coordinates are reached, if a robot gets low battery when already phisically in a room, but not yet to the given point, it believes to be still in the previous one. This happened also in the demo previously shown, but as said id didn't affect the functioning of the simulation, so it is not a big issue actually.
