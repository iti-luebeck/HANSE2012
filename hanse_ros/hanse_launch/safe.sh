#!/bin/bash
$@
echo Ensure surface
rosservice call /hanse/engine/depth/setDepth '0'
echo 60 seconds until atmega killing
sleep 60
echo killing atmega
pkill -f hanse_atmega_ros.py
