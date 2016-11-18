#!/bin/bash
#
# Setup environment to make PX4 visible to ROS/Gazebo.
#
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository

if [ "$#" -lt 1 ]
then
    SRC_DIR=/home/uav_administrator/ros_ws/src/irols
else
    SRC_DIR=$1
fi


# setup Gazebo env and update package path
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SRC_DIR}/gazebo/models
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${SRC_DIR}/Firmware/Tools/sitl_gazebo/build
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${SRC_DIR}/Firmware/Tools/sitl_gazebo/Build/msgs/
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${SRC_DIR}
#export GAZEBO_MODEL_DATABASE_URI=""
export SITL_GAZEBO_PATH=$SRC_DIR/Firmware/Tools/sitl_gazebo
