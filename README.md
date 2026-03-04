# GoCatorLineScanner

# Setup Instructions
## GO SDK
Download and build following the repository's setup instructions: [GO_SDK](https://github.com/Logan-Shi/GO_SDK)

## Include dependencies
`vcs import src < src/GoCatorLineScanner/dependencies.repos`


## Building
`colcon build && source install/setup.bash`

# Running Program and GUI
`ros2 launch gocator_application gocator_qt_gui`

This will bring up a window with various parameters that can be configured and buttons to initialize the system.

IP Address: The IP address of the camera (should be `192.168.1.10`)
Robot Speed (mm/s): The linear speed of the robot arm during scanning (currently found in the teach pendant job), if this is incorrect it will impact scanning time and scaling

Scan Distance (mm): The total distance that should be scanned (this won't impact scaling so percision is not neccessary and can be adjusted as needed to encompass the desired scanning region)

Scanning Exposure: The exposure value for the scanner (300-500 is typically sufficient)

Initialize System: Sets up the system variables and connects to the camera [MUST BE RUN FIRST]

Trigger Scan: Turns on the laser and collects data for a set amount of time (based on the robot speed and distance)