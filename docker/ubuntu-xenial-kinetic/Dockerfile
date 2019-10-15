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

ENTRYPOINT bash -c 'source /opt/ros/kinetic/setup.bash; source /root/AI-Robot-Challenge-Lab/devel/setup.bash; roslaunch sorting_demo sorting_demo.launch'

