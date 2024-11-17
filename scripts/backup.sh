#!/bin/bash

# Navigate to your repository
cd /home/mneuhaus/ros2_ws

dpkg --get-selections > ./backup/Package.list
cp -r /etc/apt/sources.list* ./backup/
cp -r /etc/apt/trusted.gpg.d/ ./backup/trusted.gpg.d/

# Add all changes
git add .

# Commit changes with a timestamp
git commit -m "Automated backup on $(date '+%Y-%m-%d %H:%M:%S')"

# Push to GitHub
git push origin master
