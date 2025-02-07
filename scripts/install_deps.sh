#!/bin/bash
echo "Installing required dependencies..."
sudo apt update
sudo apt install -y libzmq3-dev libeigen3-dev cmake g++ make python3-pip
pip3 install pyzmq

# Clone submodules
git submodule update --init --recursive

echo "Installation complete!"

