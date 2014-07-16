#!/bin/bash

SETTINGS_PATH=~/.ros_master_settings

LOCAL_MASTER_URI=http://localhost:11311
REMOTE_MASTER_URI=http://hypertaargus:11311

function source-ros-master-settings
{
    if [ -f $SETTINGS_PATH ]; then
	source $SETTINGS_PATH
    fi
}

function ros-master-local
{
    echo -e "export ROS_MASTER_URI=$LOCAL_MASTER_URI\n" > $SETTINGS_PATH
    source-ros-master-settings
}

function ros-master-remote
{
    echo -e "export ROS_MASTER_URI=$REMOTE_MASTER_URI\n" > $SETTINGS_PATH
    source-ros-master-settings
}

source-ros-master-settings