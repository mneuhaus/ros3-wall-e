#!/bin/bash

# Navigate to your repository
cd /home/mneuhaus/ros2_ws

sudo cp -r ./backup/trusted.gpg.d/* /etc/apt/trusted.gpg.d/
sudo chmod 644 /etc/apt/trusted.gpg.d/*.gpg