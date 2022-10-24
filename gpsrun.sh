#!/bin/bash
set -e

if [ -f install/setup.bash ]; then source install/setup.bash; fi
ros2 run gps_service_pkg gps_service_node