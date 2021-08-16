#!/usr/bin/env bash
#
# Typical usage: ./join.bash subt
#

BASH_OPTION=bash

IMG=jack6099boy/locobot:GPU

xhost +
containerid=$(docker ps -aqf "ancestor=${IMG}") && echo $containerid
docker exec -it \
    --privileged \
    -e DISPLAY=${DISPLAY} \
    -e LINES="$(tput lines)" \
    locobot \
    $BASH_OPTION
xhost -
