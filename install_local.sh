#!/bin/bash

# Install general Python packages
echo "Installing general Python packages..."
pip3 install --upgrade --user pip
pip3 install --user gdown black

# Ensure ~/.local/bin is in PATH
if [[ ":$PATH:" != *":$HOME/.local/bin:"* ]]; then
    echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
    export PATH=$HOME/.local/bin:$PATH
    echo "Added ~/.local/bin to PATH. Restart terminal to apply changes permanently."
fi

# Verify gdown installation
if ! command -v gdown &> /dev/null; then
    echo "Error: gdown installation failed. Exiting..."
    exit 1
fi

CRT_DIR=$(pwd)
cd int-ball2_isaac_sim

# Download the folder as a ZIP file
echo "Starting download of assets..."
gdown 1J0_ueT5xarewvBxCDvyKTfn-r0ggFGqH -O assets.zip

# Check if the download was successful
if [ ! -f "assets.zip" ]; then
    echo "Error: Download failed. Exiting..."
    exit 1
fi

# Unzip and clean up
unzip -qq assets.zip
rm assets.zip
echo "Download complete!"

cd $CRT_DIR

# Install Isaac Sim dependencies
wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | sudo tee -a /etc/apt/sources.list
sudo apt-get update
sudo apt-get install -y ros-humble-isaac-ros-nitros

# Setup Isaac Sim launcher repository
CRT_DIR=$(pwd)
cd ..

mkdir IsaacSim-ros_workspaces
git init IsaacSim-ros_workspaces
cd IsaacSim-ros_workspaces
git remote add origin https://github.com/isaac-sim/IsaacSim-ros_workspaces.git
git config core.sparseCheckout true
git sparse-checkout set humble_ws/src/isaacsim
git pull origin IsaacSim-4.2.0

cd $CRT_DIR


echo "Local installation complete!"
