#!/bin/bash

systemctl daemon-reload
systemctl disable web_rosbag_manager.service
systemctl stop web_rosbag_manager.service
rm /etc/systemd/system/web_rosbag_manager.service
