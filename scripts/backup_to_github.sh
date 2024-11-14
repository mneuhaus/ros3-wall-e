#!/bin/bash

# Navigate to your repository
cd ~/ros2_ws

# Add all changes
git add .

# Commit changes with a timestamp
git commit -m "Automated backup on $(date '+%Y-%m-%d %H:%M:%S')"

# Push to GitHub
git push origin master
