#!/bin/bash
sudo docker run --rm --gpus device=0 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env QT_X11_NO_MITSHM=1 --privileged -it ubuntu1604_sawyer /bin/bash
RES=$?

echo "RUNTIME TEST RESULT: $RES"

