#!/bin/bash

echo "Welcome! Launching real-time control of KUKA iiwa and robotic hand with Leap Motion."
gnome-terminal -e 'bash -c "roslaunch leap_rig leap_rig.launch"'&
sleep 4
gnome-terminal -e 'bash -c "cd ~/catkin_ws/src/kuka/kuka_control/scripts && /usr/local/MATLAB/R2019a/bin/matlab -nodesktop -nosplash -r realTimeController"'&
exit