#!/bin/bash
sudo startxfce4&
sleep 2;
export DISPLAY=:0
sudo docker run --rm --gpus device=0 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env QT_X11_NO_MITSHM=1 --privileged -t ubuntu1604_sawyer
RES=$?

echo "RUNTIME TEST RESULT: $RES"

