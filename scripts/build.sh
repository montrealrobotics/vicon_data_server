#!/bin/bash
echo "Building Vicon Data Server..."
mkdir -p build && cd build
cmake ..
make -j$(nproc)
echo "Build complete! Run with: ./ViconDataServer <IP> <RigidBody>"

