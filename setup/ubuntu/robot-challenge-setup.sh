#!/bin/bash

# Debug - Break on error
# set -e

# Reset the swap drive before starting to avoid virtual memory errors
sudo swapoff -a
sudo swapon -a

#
# ROS image for Sawyer development
#

# author="Paul Stubbs (pstubbs@microsoft.com)"
# description="Microsoft Robot Challenge 2018"
# version="1.0"

#
#
#
# Install Bot Framework + Emulator
echo -e '***\n***\n***\n***\nInstall bot framework + Emulator\n***\n***\n***\n***'
#
#

# Install Emulator 
sudo apt-get update
sudo apt-get -f install libindicator7
sudo apt-get -f install libappindicator1 
TEMP_DEB="$(mktemp)"
wget -O "$TEMP_DEB" 'https://github.com/Microsoft/BotFramework-Emulator/releases/download/v4.0.15-alpha/botframework-emulator_4.0.15-alpha_i386.deb'
sudo dpkg -i "$TEMP_DEB"
rm -f "$TEMP_DEB"

# Install Python 3.6
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt-get update
sudo apt-get install python3.6
sudo apt-get install python3-pip 
python3.6 -m pip install --upgrade pip

# Install Bot Framework Deps
python3.6 -m pip install --user aiohttp
python3.6 -m pip install --user requests
python3.6 -m pip install --user botbuilder.schema
python3.6 -m pip install --user botbuilder.core


# Update to lateset software lists
echo -e '***\n***\n***\n***\nUpdate to lateset software lists\n***\n***\n***\n***'

# Install some common CLI tools
sudo apt-get update -y
sudo apt-get install -y wget software-properties-common 

#
#
#
# Install ROS
echo -e '***\n***\n***\n***\nInstall ROS\n***\n***\n***\n***'
#
#
# http://sdk.rethinkrobotics.com/intera/Workstation_Setup


# Configure Ubuntu repositories. Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Install ROS Kinetic Desktop FUll
echo -e '***\n***\n***\n***\n Install ROS Kinetic Desktop FUll \n***\n***\n***\n***'

sudo apt-get update -y
sudo apt-get install -y ros-kinetic-desktop-full

# Initialize rosdep
sudo rosdep init || echo "ROSDep Already Exists."
rosdep update

# Install rosinstall
sudo apt-get install -y python-rosinstall -y

#
#
#
# Create Development Workspace
echo -e '***\n***\n***\n***\nCreate Development Workspace\n***\n***\n***\n***'
#
#
#
#Add the path to ROS
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc


# Create ROS Workspace
mkdir -p ~/ros_ws/src
cd ~/ros_ws
source /opt/ros/kinetic/setup.bash
catkin_make

#
#
#
# Install Intera SDK Dependencies
echo -e '***\n***\n***\n***\nInstall Intera SDK Dependencies\n***\n***\n***\n***'
#
#
#

# Install SDK Dependencies
# Update to lateset software lists

sudo apt-get update -y
sudo apt-get install -y 
    git-core \\
    python-argparse \\
    python-wstool \\
    python-vcstools \\
    python-rosdep \\
    ros-kinetic-control-msgs \\
    ros-kinetic-joystick-drivers \\
    ros-kinetic-xacro \\
    ros-kinetic-tf2-ros \\
    ros-kinetic-rviz \\
    ros-kinetic-cv-bridge \\
    ros-kinetic-actionlib \\
    ros-kinetic-actionlib-msgs \\
    ros-kinetic-dynamic-reconfigure \\
    ros-kinetic-trajectory-msgs \\
    ros-kinetic-rospy-message-converter

#
#
#
# Install Intera Robot SDK
echo -e '***\n***\n***\n***\nInstall Intera Robot SDK\n***\n***\n***\n***'
#
#
#
if [ ! -e ".rosinstall" ]; then
  wstool init
fi

cd ~/ros_ws/src
wstool init .
git clone https://github.com/RethinkRobotics/sawyer_robot.git
wstool merge sawyer_robot/sawyer_robot.rosinstall
wstool update

# Source ROS Setup
cd ~/ros_ws
source /opt/ros/kinetic/setup.bash
catkin_make

#
#
#
# Configure Robot Communication/ROS Workspace
echo -e '***\n***\n***\n***\nConfigure Robot Communication/ROS Workspace\n***\n***\n***\n***'
#
#
#

# Copy the intera.sh script
# The intera.sh file already exists in intera_sdk repo, 
# copy the file into your ros workspace.
cp ~/ros_ws/src/intera_sdk/intera.sh ~/ros_ws

# Update the copy of the intera.sh file
# cd ~/ros_ws
# Update ROS Distribution
sed -i 's/ros_version="indigo"/ros_version="kinetic"/' ~/ros_ws/intera.sh
# Update the ROBOTS hostname
sed -i 's/robot_hostname="robot_hostname.local"/robot_hostname="paule.local"/' ~/ros_ws/intera.sh

# TODO:// Need to figure out the docker networking to resolve hostname
# Update YOUR IP or Hostname. This must be resolvable to the Robot
# Choose one. Be sure to add or remove the leading # from the right ones
sed -i 's/your_ip="192.168.XXX.XXX"/your_ip="192.168.XXX.XXX"/' ~/ros_ws/intera.sh
#sed -i 's/#your_hostname="my_computer.local"/your_hostname="my_computer.local"/' intera.sh

# Setup and configure RVIZ
echo -e '***\n***\n***\n***\nSetup and configure RVIZ\n***\n***\n***\n***'
# TODO:// need to find a way to do this from the dockerfile


# http://sdk.rethinkrobotics.com/intera/Gazebo_Tutorial
#
#
#
# Configure Sawyer with Gazebo
#
#
#

# Install Prerequisites
# Update to lateset software lists
echo -e '***\n***\n***\n***\nUpdate to lateset software lists\n***\n***\n***\n***'

sudo apt-get update -y
sudo apt-get install -y  \
    gazebo7  \
    ros-kinetic-qt-build  \
    ros-kinetic-gazebo-ros-control  \
    ros-kinetic-gazebo-ros-pkgs  \
    ros-kinetic-ros-control  \
    ros-kinetic-control-toolbox  \
    ros-kinetic-realtime-tools  \
    ros-kinetic-ros-controllers  \
    ros-kinetic-xacro  \
    python-wstool  \
    ros-kinetic-tf-conversions  \
    ros-kinetic-kdl-parser  \
    ros-kinetic-sns-ik-lib

# Install Sawyer Simulator files
echo -e '***\n***\n***\n***\nInstall Sawyer Simulator files\n***\n***\n***\n***'

cd ~/ros_ws/src
git clone https://github.com/RethinkRobotics/sawyer_simulator.git || echo -e "***\n***\n***\n***\nsawyer_simulator,git already exists\n***\n***\n***\n***"
source /opt/ros/kinetic/setup.bash 
wstool merge sawyer_simulator/sawyer_simulator.rosinstall
wstool update

# Build the Sources
echo -e '***\n***\n***\n***\nSDK Build the Sources\n***\n***\n***\n***'
source /opt/ros/kinetic/setup.bash
cd ~/ros_ws
catkin_make
#
#
#
# Install other tools
#
#
#

# Update the machine
echo -e '***\n***\n***\n***\nUpdate the Machine\n***\n***\n***\n***'


sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get dist-upgrade -y
sudo apt-get autoremove -y
sudo apt-get autoclean -y

# Clean up all the temp files
echo -e '***\n***\n***\n***\nClean up all the temp files\n***\n***\n***\n***'
sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

