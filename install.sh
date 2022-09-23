#!/bin/bash

cat << EOF | tee /etc/systemd/system/web_rosbag_manager.service
[Unit]
Description = web_rosbag_manager launcher
After=NetworkManager-wait-online.service
Requires=NetworkManager-wait-online.service

[Service]
User=wr
ExecStart=/usr/bin/screen -S web_rosbag_manager -ADm bash -l -c 'roslaunch web_rosbag_manager web_rosbag_manager.launch'
Type=simple
Restart=always
RestartSec=1s

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable web_rosbag_manager.service
systemctl start web_rosbag_manager.service
