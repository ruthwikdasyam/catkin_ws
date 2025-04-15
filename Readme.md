# catkin_ws

This repository contains ROS packages for running a TurtleBot3 simulation with RTAB-Map integration.

## Prerequisites

- Ubuntu 20.04
- ROS Noetic

## Installation

1. Clone the repository:
   ```
   git clone https://github.com/ruthwikdasyam/catkin_ws.git
   cd catkin_ws
   ```

2. Build the workspace:
   ```
   colcon build --simulink-install
   ```
   
   If you encounter any errors or warnings, run the build command again.

## Running the Simulation

Open three separate terminal windows and run the following commands:

### Terminal 1: Launch TurtleBot3 in Gazebo

```bash
source devel/setup.bash
roslaunch turltebot_gazebo Turtlebot3_factory.launch
```
Env starts with robot at Truck Port

### Terminal 2: Launch RTAB-Map

```bash
source devel/setup.bash
roslaunch rtabmap_demos start_rtab.launch
```

### Terminal 3: Run the TurtleBot3 Control Script

```bash
source devel/setup.bash
cd src/rtabmap_ros/rtabmap_demos/scripts/
python talk_to_tb3.py
```

Give Commands - 'Travel to stairs' or 'Travel to truck port' depending on its position


## Notes

- Make sure all dependencies are properly installed before running the simulation
- If you encounter any issues, check the ROS log files for details
