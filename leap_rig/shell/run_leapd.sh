#!/bin/bash

echo "Welcome! Launching Leap Deamon."
gnome-terminal -e 'bash -c "service leapd start && LeapControlPanel"'&
exit