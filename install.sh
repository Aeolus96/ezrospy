#!/bin/bash

# set current directory as environment variable in bashrc
echo "export EZROSPY_DIRECTORY=$(pwd)" >> ~/.bashrc


# Install base packages
sudo apt install -y lsb-release curl gpg python3-wstool python3-catkin-tools


# ----- Install requirements.txt -----
pip3 install -r requirements.txt


# ----- Make python scripts executable -----
sudo chmod +x scripts/*.py
sudo chmod +x ezrospy/*.py

# ----- Clone repositories and install -----
declare -A repositories=(
    ["../rosboard"]="https://github.com/dheera/rosboard.git"
    # Add more repositories as needed
)

# Iterate over each repository and check if it already exists
for repo_dir in "${!repositories[@]}"; do
    repo_url=${repositories["$repo_dir"]}
    
    if [ -d "$repo_dir" ]; then
        echo "Repository directory $repo_dir already exists. Skipping cloning."
    else
        git clone "$repo_url" "$repo_dir"
    fi
done


# ----- Build workspace -----
# custom alias imported from another general workspace setup script, basically colcon build --symlink-install
rosbuild