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
sudo apt-get install -y wget gconf-service

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

python3.6 -m pip install --user aiohttp==3.5.1
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
sudo apt-get install -y software-properties-common curl

sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

# Configure Ubuntu repositories. Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
#sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116


# Install ROS Kinetic Desktop FUll
echo -e ${GREEN}
echo -e "***\n***\n***\n***\n Install ROS Kinetic Desktop FUll \n***\n***\n***\n***"
echo -e ${NC}

sudo apt-get update -y
sudo apt-get install -y ros-kinetic-desktop-full

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


cd ~/AI-Robot-Challenge-Lab
source /opt/ros/kinetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin build

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
sudo apt-get install -y google-chrome-stable


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
sudo apt-get install -y code

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