#!/bin/bash
set -e

if [ -f install/setup.bash ]; then source install/setup.bash; fi
ros2 launch ./launch/roscan_launch.py

