#!/bin/bash
SCRIPT=`realpath $0`
DIRNAME=`dirname $SCRIPT`

echo '[Unit]
Description="ae87-lite ros driver service"
After=network.target
Wants=roscore.service

[Service]
Type=simple
ExecStart=/bin/bash -c ". /opt/ros/noetic/setup.sh; . '$HOME'/ros_ws/devel/setup.bash;roslaunch smallBotRosDriver driver.launch"

Restart=on-failure
RestartSec=5s

[Install]
WantedBy=multi-user.target' >$DIRNAME/ae87LiteDriver.service
