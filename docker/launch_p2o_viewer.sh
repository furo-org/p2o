#!/bin/bash

xhost +local:docker
docker run -it --rm \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/path/to/your/data/dir/:/root/p2o/data" \
    kiyatdock/p2o_viewer:ubuntu24.04
xhost -local:docker

