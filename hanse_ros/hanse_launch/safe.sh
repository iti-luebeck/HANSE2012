#!/bin/bash
$@
echo Ensure surface
rosservice call /hanse/engine/depth/setDepth '0'
echo 60 seconds until roscore killing
sleep 60
echo killing roscore
pkill roscore
