#!/usr/bin/env bash

user=$(logname)

BASH_OPTION=bash

xhost +

docker run \
    -it \
    --device=/dev/dri:/dev/dri \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v "/home/$user/ICT-example:/home/isci/ICT-example" \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    --user "root:root" \
    --name locobot \
    --network host \
    --privileged \
    --security-opt seccomp=unconfined \
    iscilab/locobot:CPU \
    $BASH_OPTION
