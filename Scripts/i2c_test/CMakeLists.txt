cmake_minimum_required(VERSION 3.10)
project(robot_controller)

set(CMAKE_CXX_STANDARD 17)

# Add your source files
set(SOURCES
    main.cpp
    I2CPeripheral.cpp
    mcp23017_comm.cpp
    pca9685_comm.cpp
    RobotController.cpp
)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# Define the executable
add_executable(robot_controller ${SOURCES})

# Link against libi2c
target_link_libraries(robot_controller PRIVATE i2c)
