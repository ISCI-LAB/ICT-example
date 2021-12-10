#!/usr/bin/env bash
#
# Typical usage: ./join.bash subt
#

BASH_OPTION=bash

IMG=iscilab/locobot:GPU
containerid=$(docker ps -qf "ancestor=${IMG}") && echo $containerid

xhost +

if [[ -n "$containerid" ]]
then
    docker exec -it \
        --privileged \
        -e DISPLAY=${DISPLAY} \
        -e LINES="$(tput lines)" \
        locobot \
        $BASH_OPTION
else
    docker start -i locobot
fi
