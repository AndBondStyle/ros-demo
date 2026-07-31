#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
openvscode-server --host 0.0.0.0 --port 8000 --without-connection-token &
trap : TERM INT; sleep infinity & wait
