

# Introduction to Robotics

## Summary of technologies used

### **ROS**

ROS (Robot Operating System) is robotics middleware licensed under an open source, BSD license. Although ROS is not an Operative System, it provides libraries, hardware abstraction, device drivers, visualizers, message-passing, package management, and other tools to help software developers create robot applications. It also supports collaboration through Repositories.

Last Release: ROS Melodic Morenia. Supported on Ubuntu Artful and Bionic, along with Debian Stretch.

ROS is a distributed framework of processes (aka Nodes) that enables executables to be individually added, which makes the framework very modular. These processes can be grouped into Packages and Stacks, which can be easily shared and distributed.

Although there are some Robotics Kits available in the market, ROS is quickly becoming the new standard for both industrial and research robotics as it integrates both hardware and software for industrial applications.

**Languages:** Python 2.7

**System Requirements:** Ubuntu 16.04

**Other Supported Technologies (reference only):** C++ and Lisp

ROS supports Unix-like systems, specifically Ubuntu and Mac OS X, but the community has been adding support to Fedora, Gentoo, Arch Linux and other Linux platforms.


### **Gazebo**

Gazebo is a 3D robot simulation tool that seamlessly integrates with ROS (i.e. the Gazebo server can run in a ROS environment). Gazebo allows you to build 3D scenarios on your computer with robots, using obstacles and other objects. This allows you to test robots in complex or dangerous scenarios without any harm to the real robot. Most of the time it is faster and more cost effective to simply run a simulator instead of running the whole scenario on a real robot. It uses a physics engines called ODE (Open Dynamics Engine) for realistic movements.

Gazebo has two main components: the server, which acts like a back-end to compute all the logic for your testing scenarios, and a client, which acts as a graphical front-end. This is very useful as sometimes you might only want to execute the tests without the UI in order to speed up the execution process.

Gazebo was used to simulate the atlas robot for the Virtual Robotics Challenge (the precursor to the DARPA robotics challenge), which required participants to build a software simulation of human rescue work.

**Languages:** C++ API

**System Requirements:** Linux


### **RViz**

RViz is an open source 3D visualizer for the Robot Operating System (ROS) framework. It uses sensors data and custom visualization markers to develop robot capabilities in a 3D environment.

Features:
  - Motion planning: the process of breaking down a desired movement task into discrete motions that satisfy movement constraints and possibly optimize some aspects of the movement.
  - Object detection: visualization and recognition of objects using the camera, for example recognizing cubes of different colors.
  - Calibration: geometric camera calibration is used to estimate parameters internal to the camera that affect the image processing.
  - Debugging.
  - RViz visualization widget.
  - 3D stereo rendering: when using 2 connected cameras to record 3D images, it displays a different view to each eye so that the scene appears to have depth.

RViz provides a CLI tool that lets you execute Python or C++ scripts with controls.

### **Sawyer**

Sawyer is an integrated collaborative robot (aka cobot) solution designed with embedded vision, smart swappable grippers, and high resolution force control. The robot's purpose is to automate specific repetitive industrial tasks. It comes with an arm that has a gripper which can be easily replaced by one of the available options from the ClickSmart Gripper Kit.

Features:
  - Sawyer comes with highly sensitive torque sensors embedded into every joint. This allows you to control force where delicate part insertion is critical or use force if needed. It can maneuver into tight spaces and it has a long reach of up to 1260 mm (50 inches).
  - Comes with an embedded vision system used for the robot positioning. It also allows external vision systems like cameras.
  - Fast to deploy as many pieces are plug & play with integrated sensors.

### **MoveIt!**

MoveIt is software for motion and path planning. Users can access actions and services using: C++, Python, or through a GUI.

Features:
  - Manipulation
  - Motion planning: the process of breaking down a desired movement task into discrete motions that satisfy movement constraints and possibly optimize some aspects of the movement.
  - 3D perception: allows the robot to extract 3D information from the world and their own movements so that they can accomplish navigation and manipulation tasks.
  - Kinematics: geometry for moving the arms.
  - Control and navigation: underneath it uses OMPL (Open Motion Planning Library) and requires a controller to send messages to the hardware. MoveIt provides a Fake Controller to interact with the hardware using ROS messages but you can replace it with your own plugin which controls the robot if needed.

The planning scene feature allows you to monitor the state, sensor and world geometry information.