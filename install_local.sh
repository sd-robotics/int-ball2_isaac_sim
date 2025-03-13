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

# Download the folder as a ZIP file
echo "Starting download of assets..."
gdown 19A5S4MTq3EK9BbdZ2XaRgkUJNvmvWNTu -O assets.zip

# Check if the download was successful
if [ ! -f "assets.zip" ]; then
    echo "Error: Download failed. Exiting..."
    exit 1
fi

# Unzip and clean up
unzip -qq assets.zip
rm assets.zip
echo "Download complete!"

echo "Local installation complete!"
