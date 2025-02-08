#!/bin/bash
set -ex  # Exit on failure and print all commands

echo "Building Vicon Data Server..."

# Step 1: Build cppzmq (external dependency) without tests
echo "Building cppzmq..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/../external/cppzmq"
mkdir -p build && cd build
cmake .. -DCPPZMQ_BUILD_TESTS=OFF
make -j$(nproc)
sudo make install
echo "cppzmq build complete!"

# Step 2: Build Vicon Data Server
echo "Building Vicon Data Server..."
cd "$SCRIPT_DIR/../"
mkdir -p build && cd build
cmake .. -DCPPZMQ_BUILD_TESTS=OFF

# Debug dependencies
echo "CMake dependencies found:"
cmake --build . --target help

# Build the project (disable parallel building for debugging)
make -j1
echo "Build complete! Run with: ./ViconDataServer <IP> <RigidBody>"

