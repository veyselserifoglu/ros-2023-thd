# Maze Runner - Manual Exploration

Simple TurtleBot3 manual maze exploration with path recording.

## Features
- Manual robot control with teleop
- Automatic path recording during exploration
- Simple maze environment in Gazebo

## Quick Start

### 1. Build and Source
```bash
cd /turtlebot3_ws
catkin_make
source devel/setup.bash
```

### 2. Set Robot Model
```bash
export TURTLEBOT3_MODEL=burger
```

### 3. Launch Manual Exploration
```bash
roslaunch maze_runner manual_exploration.launch
```
This starts:
- Gazebo with maze world
- TurtleBot3 robot
- SLAM mapping
- Path recorder

### 4. Manual Control (Second Terminal)
```bash
cd /turtlebot3_ws
source devel/setup.bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Control keys:
- `w`: forward
- `a`: turn left  
- `s`: stop
- `d`: turn right
- `x`: backward

## Path Recording
- Automatically records your exploration path
- Records poses every 0.5m or 20° rotation
- Path stored in ROS parameter: `/path_recorder_node/recorded_path`

## Check Recorded Path
```bash
rosparam get /path_recorder_node/path_length
```

That's it! Simple manual exploration with automatic path recording.
