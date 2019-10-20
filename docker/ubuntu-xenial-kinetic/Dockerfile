FROM ubuntu:xenial
ENV DEBIAN_FRONTEND noninteractive

# install packages
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*


RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y git python sudo gconf-service gconf2 libxss1 libnss3 libnotify4 software-properties-common apt-utils

RUN bash -c 'cd $HOME; git clone https://github.com/pabloinigoblasco/AI-Robot-Challenge-Lab.git --recurse-submodules'

RUN bash -c 'export DEBIAN_FRONTEND=noninteractive; $HOME/AI-Robot-Challenge-Lab/setup/robot-challenge-setup.sh'
RUN bash -c 'cd $HOME/AI-Robot-Challenge-Lab; git pull'
RUN bash -c 'cd $HOME/AI-Robot-Challenge-Lab; git submodule init; git submodule update --recursive'
RUN bash -c 'cd $HOME/AI-Robot-Challenge-Lab; catkin build'

COPY intera.sh /root/AI-Robot-Challenge-Lab
COPY run.sh /root/AI-Robot-Challenge-Lab/run.sh
#ENTRYPOINT bash
#ENTRYPOINT bash -c 'cd /root/AI-Robot-Challenge-Lab; source /opt/ros/kinetic/setup.bash; ./intera.sh sim ./run.sh'

