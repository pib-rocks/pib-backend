#!/bin/bash

USER=$1
WORK_DIR=$2

cat << EOF
[Unit]
Description=Startup script for ROS and Cerebra
After=network.target

[Service]
Type=simple
User=$USER
ExecStart=/usr/bin/bash $WORK_DIR/ros_camera_boot.sh

[Install]
WantedBy=multi-user.target
EOF