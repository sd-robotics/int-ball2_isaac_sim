#!/bin/bash
set -e

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
gdown 1QsfOaWS-0is1ym5XxRF_tqr1Yi3pCfUz -O assets.zip

# Check if the download was successful
if [ ! -f "assets.zip" ]; then
    echo "Error: Download failed. Exiting..."
    exit 1
fi

# Unzip and clean up
unzip -qq assets.zip
rm assets.zip
echo "Download complete!"

cd "$CRT_DIR"

# Clone ib2 interface repository
if [ ! -d "ib2_interfaces" ]; then
    git clone https://github.com/sd-robotics/ib2_interfaces.git
else
    echo "ib2_interfaces directory already exists, skipping clone."
fi


# Install Isaac Sim dependencies
wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | sudo tee -a /etc/apt/sources.list
sudo apt-get update
sudo apt-get install -y ros-humble-isaac-ros-nitros

# Setup Isaac Sim launcher repository
cd ..
if [ ! -d "IsaacSim-ros_workspaces" ]; then
    mkdir IsaacSim-ros_workspaces
    cd IsaacSim-ros_workspaces
    git init
    git remote add origin https://github.com/isaac-sim/IsaacSim-ros_workspaces.git
    git config core.sparseCheckout true
    git sparse-checkout set humble_ws/src/isaacsim
    git pull origin main
    cd "$CRT_DIR"
else
    echo "IsaacSim-ros_workspaces directory already exists, skipping clone."
    cd "$CRT_DIR"
fi


echo "Local installation complete!"
