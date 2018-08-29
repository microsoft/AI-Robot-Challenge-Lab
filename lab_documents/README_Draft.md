# AI Robot Challenge

Introduction and what will be achieved.. TODO

# Getting started with robotics

## Introduction to technologies used

### **ROS**

ROS (Robot Operating System) is robotics middleware licensed under an open source, BSD license. Althought ROS is not an Operative System, it provides libraries, hardware abstraction, device drivers, visualizers, message-passing, package management, and other tools to help software developers create robot applications.
Last Release: ROS Melodic Morenia. Supported on Ubuntu Artful and Bionic, along with Debian Stretch. 
Supports collaboration throught Repositories.
ROS is a distributed framework of processes (aka Nodes) that enables executables to be individually added, which makes the framework very modular. These processes can be grouped into Packages and Stacks, which can be easily shared and distributed.

**Languages:**  Python, C++, and Lisp

**System Requirements:** supports Unix-based systems, primarly Ubuntu and Mac OS X systems, but the community has been adding support to Fedora, Gentoo, Arch Linux and other Linux platforms.
Althought there are some Robotics Kits available in the market, ROS is quickly becoming the new standard for both industrial and research robotics as it integrates both hardware and software in their solution for industrial applications.

### **Gazebo**

Gazebo is a 3D robot simulation tool that seamlessly integrates with ROS, which allows to run the Gazebo server in a ROS environment. Gazebo allows to build 3D scenarios on your computer with robots, using obstacles and other objects. This allows to test robots in complex or dangerous scenarios without any harm to the real robot. Most of the time it is faster and cost effective to simply run a simulator instead of starting the whole scenario on a real robot. The testing models used by the simulators can be created using XML or a graphical model editor. It uses a physics engines for realistic movements called ODE (Open Dynamics Engine).
Gazebo has two main components: the server which acts like a back-end to compute all the logic for your testing scenarios and a client which works as a graphical front-end. This is very useful as sometimes you might only want to execute the tests without the UI in order to speed up the execution process.
Gazebo was used to simulate the atlas robot for the Virtual Robotics Challenge (the precursor to the DARPA robotics challenge), which required to build a software simulation of humanoid rescue work.
There are other commercial versions for robotics simulations but Gazebo is a strong competitor that is free to use under Apache 2.0 license.

**Languages:** C++ API

**System Requirements:** Linux

**Robotics Middleware support:** ROS, Player, Sockets (protobuf messages)

### **RViz**

RViz is an Open Source 3D visualizer for the Robot Operating System (ROS) framework.
Uses sensors data and custom visualization markers to develop robot capabilities in a 3d environment.
Features:
  - Motion planning
  - Object detection
  - Calibration
  - Debugging
  - RViz visualization widget
  - 3D stereo rendering
RViz provides a CLI tool that lets you execute python or c++ scripts with controls.

### **Sawyer**

Sawyer is an integrated collaborative robot (aka cobot) solution designed with embedded vision, ClickSmart grippers, and high resolution force control. The robot purpose is to automate specific industrial repetive tasks, it comes with an arm that has a gripper which can be easily replaced by one of the available options from the ClickSmart Gripper Kit.
Features:
  - Sawyer comes with highly sensitive torque sensors embedded into every joint, this allows you to control force where delicate part insertion is critical, or use force if needed. It can maneuver into tight spaces and it has a long reach of 1260 mm (max).
  - Comes with an embedded vision system used for the robot positioning, it also allows external vision systems like cameras.
  - The software that comes with the robot is continuosly updated.
  - Fast to deploy as many pieces are plug&play, ready to use and with integrated sensors.

### **MoveIt!**

MoveIt is software for motion and path planning. Users can access actions and services using: C++, Python, Through a GUI.
Features:
  - Motion planning
  - Manipulation
  - 3D perception
  - Kinematics
  - Control and navigation
Underneath it uses OMPL (Open Motion Planning Library) and requires a controller to send messages to the hardware. MoveIt provides a Fake Controller to interact with the hardware using ROS messages but you can replace the fake robot controller in MoveIt with your own plugin which controls the robot if needed.
The planning scene feature allows to monitor the state, sensor and world geometry information.

## Getting started  

# Bringing your robot to life

# Making your robot intelligent with Microsoft AI


