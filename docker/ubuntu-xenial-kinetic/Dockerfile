FROM ubuntu:16.04

MAINTAINER Pablo Inigo <pblasco@plainconcepts.com>

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-mark hold iptables && \
    apt-get -y dist-upgrade && apt-get autoremove -y && apt-get clean
RUN apt-get install -y dbus-x11 procps psmisc apt-utils x11-utils x11-xserver-utils kmod xz-utils && \
    apt-get -y install dialog git vim software-properties-common debconf-utils wget curl apt-transport-https bzip2 zip iputils-ping telnet less

# OpenGL / MESA
RUN apt-get install -y mesa-utils mesa-utils-extra libxv1 

# Language/locale settings
#   replace en_US by your desired locale setting, 
#   for example de_DE for german.
ENV LANG en_US.UTF-8
RUN echo $LANG UTF-8 > /etc/locale.gen
RUN apt-get install -y locales && update-locale --reset LANG=$LANG

# some utils to have proper menus, mime file types etc.
RUN apt-get install -y --no-install-recommends xdg-utils xdg-user-dirs \
    menu menu-xdg mime-support desktop-file-utils

# Install fonts
# We just need to drop these into the shared fonts folder, the font cache will be rebuilt when we add the desktop environments
#RUN wget https://dl.1001fonts.com/mclaren.zip && \
#    unzip -d /usr/share/fonts/truetype/mclaren mclaren.zip && \
#    rm mclaren.zip && \
#    mkdir -p /usr/share/fonts/truetype/monofur && \
#    cd /usr/share/fonts/truetype/monofur && \
#    wget https://github.com/ryanoasis/nerd-fonts/blob/master/patched-fonts/Monofur/Regular/complete/monofur%20Nerd%20Font%20Complete%20Mono.ttf?raw=true -O '/usr/share/fonts/truetype/monofur/monofur Nerd Font Complete Mono.ttf' \
#         https://github.com/ryanoasis/nerd-fonts/blob/master/patched-fonts/Monofur/Regular/complete/monofur%20Nerd%20Font%20Complete.ttf?raw=true -O '/usr/share/fonts/truetype/monofur/monofur Nerd Font Complete.ttf' \
#         https://github.com/ryanoasis/nerd-fonts/blob/master/patched-fonts/Monofur/Italic/complete/monofur%20italic%20Nerd%20Font%20Complete%20Mono.ttf?raw=true -O '/usr/share/fonts/truetype/monofur/monofur italic Nerd Font Complete Mono.ttf' \
#         https://github.com/ryanoasis/nerd-fonts/blob/master/patched-fonts/Monofur/Italic/complete/monofur%20italic%20Nerd%20Font%20Complete.ttf?raw=true -O '/usr/share/fonts/truetype/monofur/monofur italic Nerd Font Complete.ttf' \
#         https://github.com/ryanoasis/nerd-fonts/blob/master/patched-fonts/Monofur/Bold/complete/monofur%20bold%20Nerd%20Font%20Complete%20Mono.ttf?raw=true -O '/usr/share/fonts/truetype/monofur/monofur bold Nerd Font Complete Mono.ttf.ttf' \
#         https://github.com/ryanoasis/nerd-fonts/blob/master/patched-fonts/Monofur/Bold/complete/monofur%20bold%20Nerd%20Font%20Complete.ttf?raw=true -O '/usr/share/fonts/truetype/monofur/monofur bold Nerd Font Complete.ttf' 

RUN apt-get install sudo
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
RUN sudo apt-get update
RUN ln -fs /usr/share/zoneinfo/US/Pacific-New /etc/localtime && apt-get install tzdata


RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y git python sudo gconf-service gconf2 libxss1 libnss3 libnotify4 software-properties-common apt-utils

RUN apt-get install -y chromium-browser sudo mesa-utils

# create user
RUN bash -c "useradd -ms /bin/bash nvagent"
RUN usermod -aG sudo nvagent
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER nvagent
WORKDIR /home/nvagent
RUN echo "source /opt/ros/kinetic/setup.bash" >> .bashrc

# downloading repo and configuring environment
RUN bash -c 'cd $HOME; git clone https://github.com/pabloinigoblasco/AI-Robot-Challenge-Lab.git --recurse-submodules'

RUN bash -c 'export DEBIAN_FRONTEND=noninteractive; $HOME/AI-Robot-Challenge-Lab/setup/robot-challenge-setup.sh'
WORKDIR /home/nvagent/AI-Robot-Challenge-Lab/

RUN git pull
RUN git submodule init; git submodule update --recursive
RUN bash -c 'catkin build'

RUN sudo apt-get install -y x11-apps python-pip
RUN python -m pip install requests --user

COPY runtime_test.sh runtime_test.sh
COPY runtime_test.py runtime_test.py
RUN sudo echo "export SVGA_VGPU10=0" >> ~/.bashrc
RUN sudo wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN sudo apt-get update
RUN sudo apt-get install -y gazebo7


#ENTRYPOINT /bin/bash
ENTRYPOINT /home/nvagent/AI-Robot-Challenge-Lab/runtime_test.sh sim

