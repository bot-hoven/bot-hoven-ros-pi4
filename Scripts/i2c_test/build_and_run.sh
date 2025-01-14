#!/bin/bash

# Navigate to the parent directory
# cd ..

# Remove the old build folder if it exists
if [ -d "build" ]; then
    echo "Removing old build directory..."
    rm -rf build
fi

# Create a new build folder and navigate into it
echo "Creating new build directory..."
mkdir build && cd build

# Run cmake
echo "Running cmake..."
cmake ..

# Build the project
echo "Building the project..."
make

# Run the executable
echo "Running the robot controller..."
./robot_controller
