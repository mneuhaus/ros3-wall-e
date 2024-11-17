#!/bin/bash

# Navigate to your repository
cd /home/mneuhaus/ros2_ws

sudo chmod -R 775 /home/mneuhaus/ros2_ws
sudo chown -R mneuhaus:mneuhaus /home/mneuhaus/ros2_ws

sudo dpkg --get-selections > ./backup/Package.list
sudo cp -r /etc/apt/sources.list* ./backup/
sudo cp -r /etc/apt/trusted.gpg.d/ ./backup/trusted.gpg.d/

# Add all changes
git add .

# Commit changes with a timestamp
git commit -m "Automated backup on $(date '+%Y-%m-%d %H:%M:%S')"

# Push to GitHub
git push origin master
