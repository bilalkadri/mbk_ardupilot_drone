#!/bin/bash
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -L KIET -f gazebo-iris --console  --out 127.0.0.1:14650

# Add KIET to the locations.txt file:
# gedit ~/ardupilot/Tools/autotest/locations.txt
# Paste this line at the end: KIET=24.7944000,67.1352048,5,0