#!/bin/bash

if [ $# -eq 0 ]
    then
        echo "Please give the path to PX4 Firmware root dir"
        return 1
    else
        if [ ! -d "$1/build_posix_sitl_default" ]
            then
                cd $1 && make posix_sitl_default && cd -
        fi
        if [ -z "$(ls -A -- "$1/build_posix_sitl_default/build_gazebo")" ]
            then
                cd $1/build_posix_sitl_default && make sitl_gazebo && cd -
        fi
        source $1/Tools/setup_gazebo.bash $1 $1/build_posix_sitl_default
        export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find irols)/gazebo/models
        export GAZEBO_WORLD_PATH=$GAZEBO_WORLD_PATH:$(rospack find irols)/gazebo/worlds
        export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$1:$1/Tools/sitl_gazebo
        #export ROS_MASTER_URI=http://excalibur.ocas.uc.edu:11311
        #export ROS_HOSTNAME=excalibur.ocas.uc.edu
        #export GAZEBO_MASTER_URI=http://excalibur.ocas.uc.edu:11345
fi


