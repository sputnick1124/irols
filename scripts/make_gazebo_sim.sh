#!/usr/bin/env sh
cd /home/uav_master/Documents/pixhawk/Firmware
no_sim=1 make posix_sitl_default gazebo
cd -
