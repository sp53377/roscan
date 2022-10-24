#!/bin/bash
set -e

if [ -f install/setup.bash ]; then source install/setup.bash; fi
ros2 run canplay_pkg canplay_node ./examples/SampleLog.txt

