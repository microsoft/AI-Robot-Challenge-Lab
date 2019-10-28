FROM nvidia/cudagl:9.2-base-ubuntu16.04

ENV DEBIAN_FRONTEND noninteractive

# install packages
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

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

RUN sudo apt-get install -y x11-apps
COPY runtime_test.sh runtime_test.sh
COPY runtime_test.py runtime_test.py

ENTRYPOINT /home/nvagent/AI-Robot-Challenge-Lab/runtime_test.sh sim
#ENTRYPOINT bash -c 'cd /root/AI-Robot-Challenge-Lab; source /opt/ros/kinetic/setup.bash; ./intera.sh sim ./run.sh'

