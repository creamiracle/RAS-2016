#!/bin/sh
sleep 3
rosrun dynamic_reconfigure dynparam set /camera/driver color_enable_auto_white_balance $1
rosrun dynamic_reconfigure dynparam set /camera/driver color_white_balance $2
