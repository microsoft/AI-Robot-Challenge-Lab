#!/bin/bash

# Debug - Break on error
set -e

# Hack: Clear any file locks
sudo rm /var/lib/apt/lists/lock

# Color it
NC='\033[0m' # No Color
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'

# Time it
scriptstart=$(date +%s)

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
echo -e ${GREEN}
echo -e "***\n***\n***\n***\nInstall bot framework + Emulator\n***\n***\n***\n***"
echo -e ${NC}
# Time it
start=$(date +%s)
#
#

sudo apt-get update
# install unmet dependencies
sudo apt-get -f install -y
sudo apt-get -f install libindicator7 -y
sudo apt-get -f install libappindicator1 -y
# install any unmet dependencies
sudo apt-get -f install -y

TEMP_DEB="$(mktemp)"
wget -O "$TEMP_DEB" 'https://roboticslabstorage.blob.core.windows.net/github-assets/botframework-emulator_4.0.15-alpha_amd64.deb'
sudo dpkg -i "$TEMP_DEB"
rm -f "$TEMP_DEB"
# install any unmet dependencies
sudo apt-get -f install -y
sudo apt autoremove -y

# Install Python 3.6
echo -e ${GREEN}
echo -e "***\n***\n***\n***\nInstall Python 3.6\n***\n***\n***\n***"
echo -e ${NC}
sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt-get update -y
sudo apt-get install -y python3.6
sudo apt-get install -y python3-pip
# install any unmet dependencies
sudo apt-get -f install -y
python3.6 -m pip install --upgrade pip

# Install Bot Framework Deps
echo -e ${GREEN}
echo -e "***\n***\n***\n***\nInstall bot framework Dependencies\n***\n***\n***\n***"
echo -e ${NC}

python3.6 -m pip install --user aiohttp
python3.6 -m pip install --user requests
python3.6 -m pip install --user botbuilder.schema
python3.6 -m pip install --user botbuilder.core

# Time it
end=$(date +%s)
runtime=$(python -c "print '%u:%02u' % ((${end} - ${start})/60, (${end} - ${start})%60)")
echo -e ${BLUE}
echo -e "Elapsed Time: ${runtime}"
echo -e ${NC}

#
#
#
# Install ROS
echo -e ${GREEN}
echo -e "***\n***\n***\n***\nInstall ROS\n***\n***\n***\n***"
echo -e ${NC}
# Time it
start=$(date +%s)
#
#
# http://sdk.rethinkrobotics.com/intera/Workstation_Setup

# Update to lateset software lists
echo -e ${GREEN}
echo -e "***\n***\n***\n***\nUpdate to lateset software lists\n***\n***\n***\n***"
echo -e ${NC}

# Install some common CLI tools
sudo apt-get update -y
sudo apt-get install -y wget software-properties-common 

# Configure Ubuntu repositories. Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Install ROS Kinetic Desktop FUll
echo -e ${GREEN}
echo -e "***\n***\n***\n***\n Install ROS Kinetic Desktop FUll \n***\n***\n***\n***"
echo -e ${NC}

sudo apt-get update -y
sudo apt-get install -y ros-kinetic-desktop-full --allow-unauthenticated

# Initialize rosdep
sudo rosdep init || echo -e "${YELLOW}ROSDep Already Exists.${NC}"
rosdep update

# Install rosinstall
sudo apt-get install -y python-rosinstall -y

# Time it
end=$(date +%s)
runtime=$(python -c "print '%u:%02u' % ((${end} - ${start})/60, (${end} - ${start})%60)")
echo -e ${BLUE}
echo -e "Elapsed Time: ${runtime}"
echo -e ${NC}


#
#
#
# Create Development Workspace
echo -e ${GREEN}
echo -e "***\n***\n***\n***\nCreate Development Workspace\n***\n***\n***\n***"
echo -e ${NC}
# Time it
start=$(date +%s)
#
#
#
#Add the path to ROS
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc


# Create ROS Workspace
mkdir -p ~/ros_ws/src
cd ~/ros_ws
source /opt/ros/kinetic/setup.bash
catkin_make

# Time it
end=$(date +%s)
runtime=$(python -c "print '%u:%02u' % ((${end} - ${start})/60, (${end} - ${start})%60)")
echo -e ${BLUE}
echo -e "Elapsed Time: ${runtime}"
echo -e ${NC}


#
#
#
# Install Intera SDK Dependencies
echo -e ${GREEN}
echo -e "***\n***\n***\n***\nInstall Intera SDK Dependencies\n***\n***\n***\n***"
echo -e ${NC}
# Time it
start=$(date +%s)
#
#
#

# Install SDK Dependencies
# Update to lateset software lists

sudo apt-get update
sudo apt-get install -y \
  git-core \
  python-argparse \
  python-wstool \
  python-vcstools \
  python-rosdep \
  ros-kinetic-control-msgs \
  ros-kinetic-joystick-drivers \
  ros-kinetic-xacro \
  ros-kinetic-tf2-ros \
  ros-kinetic-rviz \
  ros-kinetic-cv-bridge \
  ros-kinetic-actionlib \
  ros-kinetic-actionlib-msgs \
  ros-kinetic-dynamic-reconfigure \
  ros-kinetic-trajectory-msgs \
  ros-kinetic-rospy-message-converter

# Time it
end=$(date +%s)
runtime=$(python -c "print '%u:%02u' % ((${end} - ${start})/60, (${end} - ${start})%60)")
echo -e ${BLUE}
echo -e "Elapsed Time: ${runtime}"
echo -e ${NC}

#
#
#
# Install Intera Robot SDK
echo -e ${GREEN}
echo -e "***\n***\n***\n***\nInstall Intera Robot SDK\n***\n***\n***\n***"
echo -e ${NC}
# Time it
start=$(date +%s)
#
#
#

cd ~/ros_ws/src
wstool init .
git clone https://github.com/RethinkRobotics/sawyer_robot.git
wstool merge sawyer_robot/sawyer_robot.rosinstall
wstool update

# Source ROS Setup
cd ~/ros_ws
source /opt/ros/kinetic/setup.bash
catkin_make

# Time it
end=$(date +%s)
runtime=$(python -c "print '%u:%02u' % ((${end} - ${start})/60, (${end} - ${start})%60)")
echo -e ${BLUE}
echo -e "Elapsed Time: ${runtime}"
echo -e ${NC}


#
#
#
# Configure Robot Communication/ROS Workspace
echo -e ${GREEN}
echo -e "***\n***\n***\n***\nConfigure Robot Communication/ROS Workspace\n***\n***\n***\n***"
echo -e ${NC}
# Time it
start=$(date +%s)
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
echo -e ${GREEN}
echo -e "***\n***\n***\n***\nSetup and configure RVIZ\n***\n***\n***\n***"
echo -e ${NC}
# TODO:// need to do this still

# Time it
end=$(date +%s)
runtime=$(python -c "print '%u:%02u' % ((${end} - ${start})/60, (${end} - ${start})%60)")
echo -e ${BLUE}
echo -e "Elapsed Time: ${runtime}"
echo -e ${NC}


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
echo -e ${GREEN}
echo -e "***\n***\n***\n***\nUpdate to lateset software lists\n***\n***\n***\n***"
echo -e ${NC}
# Time it
start=$(date +%s)
#
#
#
sudo apt-get update -y
sudo apt-get install -y \
  gazebo7 \
  ros-kinetic-qt-build \
  ros-kinetic-gazebo-ros-control \
  ros-kinetic-gazebo-ros-pkgs \
  ros-kinetic-ros-control \
  ros-kinetic-control-toolbox \
  ros-kinetic-realtime-tools \
  ros-kinetic-ros-controllers \
  ros-kinetic-xacro \
  python-wstool \
  ros-kinetic-tf-conversions \
  ros-kinetic-kdl-parser \
  ros-kinetic-sns-ik-lib

# Install Sawyer Simulator files
echo -e ${GREEN}
echo -e "***\n***\n***\n***\nInstall Sawyer Simulator files\n***\n***\n***\n***"
echo -e ${NC}

cd ~/ros_ws/src
if [ ! -d src/sawyer_simulator ]
then
  # folder does not exist so clone the repo
	echo -e ${YELLOW}
  echo "~/ros_ws/src/sawyer_simulator folder does not exist, cloning now"
  echo -e ${NC}
  git clone https://github.com/RethinkRobotics/sawyer_simulator.git
else
  # folder does exist so pull to update
	echo -e ${YELLOW}
  echo "~/ros_ws/src/sawyer_simulator folder already exists, updating now"
  echo -e ${NC}
  cd ~/ros_ws/src/sawyer_simulator
  git pull
  cd ~/ros_ws/src
fi

source /opt/ros/kinetic/setup.bash 
wstool merge sawyer_simulator/sawyer_simulator.rosinstall
wstool update

# Build the Sources
echo -e ${GREEN}
echo -e "***\n***\n***\n***\nSDK Build the Sources\n***\n***\n***\n***"
echo -e ${NC}
source /opt/ros/kinetic/setup.bash
cd ~/ros_ws
catkin_make

# Time it
end=$(date +%s)
runtime=$(python -c "print '%u:%02u' % ((${end} - ${start})/60, (${end} - ${start})%60)")
echo -e ${BLUE}
echo -e "Elapsed Time: ${runtime}"
echo -e ${NC}


#
#
#
# Install other tools
#
#
#

#
#
#
# Install Chromium Browser
echo -e ${GREEN}
echo -e "***\n***\n***\n***\nInstall Chromium Browser\n***\n***\n***\n***"
echo -e ${NC}
# Time it
start=$(date +%s)
#
#
#
#https://www.linuxbabe.com/ubuntu/install-google-chrome-ubuntu-16-04-lts
sudo su -c "echo 'deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main' >> /etc/apt/sources.list"
TEMP_DEB="$(mktemp)"
wget -O "$TEMP_DEB" 'https://dl.google.com/linux/linux_signing_key.pub'
sudo apt-key add "$TEMP_DEB"
rm -f "$TEMP_DEB"
sudo apt-get update
sudo apt-get install google-chrome-stable


# Time it
end=$(date +%s)
runtime=$(python -c "print '%u:%02u' % ((${end} - ${start})/60, (${end} - ${start})%60)")
echo -e ${BLUE}
echo -e "Elapsed Time: ${runtime}"
echo -e ${NC}


#
#
#
# Install Visual Studio Code
echo -e ${GREEN}
echo -e "***\n***\n***\n***\nInstall Visual Studio Code\n***\n***\n***\n***"
echo -e ${NC}
# Time it
start=$(date +%s)
#
#
#
#https://tecadmin.net/install-visual-studio-code-editor-ubuntu/
sudo su -c "echo 'deb [arch=amd64] http://packages.microsoft.com/repos/vscode stable main' >> /etc/apt/sources.list.d/vscode.list"
curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
sudo mv microsoft.gpg /etc/apt/trusted.gpg.d/microsoft.gpg
sudo apt-get update
sudo apt-get install code

# Time it
end=$(date +%s)
runtime=$(python -c "print '%u:%02u' % ((${end} - ${start})/60, (${end} - ${start})%60)")
echo -e ${BLUE}
echo -e "Elapsed Time: ${runtime}"
echo -e ${NC}

#
#
#
# Update the machine
echo -e ${GREEN}
echo -e "***\n***\n***\n***\nUpdate the Machine\n***\n***\n***\n***"
echo -e ${NC}
#
#
#
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get dist-upgrade -y
sudo apt-get autoremove -y
sudo apt-get autoclean -y

# Time it
start=${scriptstart}
end=$(date +%s)
runtime=$(python -c "print '%u:%02u' % ((${end} - ${start})/60, (${end} - ${start})%60)")
echo -e ${BLUE}
echo -e "Total Elapsed Time: ${runtime}"
echo -e ${NC}
