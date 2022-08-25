#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/noetic/setup.bash"
export ROS_MASTER_URI=http://10.0.0.143:11311/
exec "$@"
