# Simple Mobile Robot ROS Package

## Overview

This README provides instructions on how to test and run the `simple_mobile_robot` ROS package. This package simulates a mobile robot in ROS Noetic and visualizes the robot's chassis, caster, and wheels. It includes a launch file for visualization in RViz.

## Prerequisites

- Ubuntu 20.04 LTS
- ROS Noetic

If ROS Noetic is not installed, please follow the [official installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu).

## Building the Package

Follow these steps to build the `simple_mobile_robot` package in your catkin workspace:

1. Navigate to your catkin workspace directory:

   ```sh
   cd ~/catkin_ws
   ```

2. Build the workspace:

   ```sh
   catkin_make
   ```

3. Source the workspace setup file:

   ```sh
   source devel/setup.bash
   ```

## Running the Package

To launch the package and visualize the robot in RViz, run:

```sh
roslaunch simple_mobile_robot robot_visualization.launch
```

## Testing the Code

You can adjust the `radius` and `angular_speed` parameters within the `robot_state_publisher.py` script to test different scenarios:

- **Radius**: Changes the size of the robot's circular path.
- **Angular Speed**: Controls the robot's speed around the path.

To modify these parameters, edit the `robot_state_publisher.py` script and rebuild the package using `catkin_make`.

## Customization

Customize the robot's appearance by modifying the `chasis`, `caster`, `wheel_r`, and `wheel_l` variables within the script to change scale and color properties.

Rebuild your workspace after making changes to apply them.

## Troubleshooting

If you encounter issues, ensure the following:

- ROS Noetic is correctly installed and sourced.
- The workspace is built without errors.
- The `robot_visualization.launch` file exists in the package's `launch` directory.
- You have sourced your workspace's `devel/setup.bash` file.

## Support

For further assistance, please refer to ROS Answers or the official ROS documentation.