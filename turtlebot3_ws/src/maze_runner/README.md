# Maze Runner - Manual Exploration & Autonomous Retraction

Clean TurtleBot3 maze navigation system with manual exploration and service-triggered autonomous retraction.

## Features
- **Manual WASD control** for maze exploration
- **Automatic path recording** during manual navigation  
- **Service-triggered A* retraction** for autonomous return to start
- **SLAM mapping** for environment visualization

## Simple Workflow

### Step 1: Launch Manual Exploration
```bash
cd /turtlebot3_ws
source devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch maze_runner manual_exploration.launch
```
This starts:
- Gazebo with maze world
- TurtleBot3 robot at starting position
- SLAM mapping with RViz visualization
- Path recorder (automatically recording waypoints)
- Navigation stack (move_base) ready for retraction

### Step 2: Manual Control (New Terminal)
```bash
cd /turtlebot3_ws
source devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

**Controls:**
- `w` - Move forward
- `a` - Turn left  
- `s` - Stop
- `d` - Turn right
- `x` - Move backward

### Step 3: Start A* Retractor (New Terminal)
```bash
cd /turtlebot3_ws
source devel/setup.bash
export TURTLEBOT3_MODEL=burger
python3 /turtlebot3_ws/src/maze_runner/scripts/simple_astar_retractor.py
```

### Step 4: Trigger Autonomous Retraction (New Terminal)
```bash
cd /turtlebot3_ws
source devel/setup.bash
rosservice call /simple_astar_retractor/trigger_retraction "{}"
```

The robot will autonomously navigate back to the starting position using the recorded waypoints.

## System Components

### Essential Files
- `launch/manual_exploration.launch` - Main system launcher (Gazebo + SLAM + path recording)
- `scripts/path_recorder.py` - Records waypoints during manual exploration
- `scripts/simple_astar_retractor.py` - Autonomous retraction service
- `launch/smooth_move_base.launch` - Navigation configuration for smooth movement

### Path Recording
- **Automatic**: Records waypoints as you explore manually
- **Distance threshold**: 0.8m between waypoints
- **Angle threshold**: 0.5 radians (29°) rotation change
- **Storage**: ROS parameter `/path_recorder_node/recorded_path`

### A* Retraction
- **Trigger**: ROS service `/simple_astar_retractor/trigger_retraction`
- **Algorithm**: Uses recorded waypoints for autonomous navigation
- **Navigation**: move_base with obstacle avoidance

## Quick Commands

```bash
# Launch exploration
roslaunch maze_runner manual_exploration.launch

# Manual control (new terminal)
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

# Start retractor (new terminal)  
python3 /turtlebot3_ws/src/maze_runner/scripts/simple_astar_retractor.py

# Trigger retraction (new terminal)
rosservice call /simple_astar_retractor/trigger_retraction "{}"

# Monitor recorded path
rosparam get /path_recorder_node/recorded_path
```

## Troubleshooting

- **No path recorded**: Move robot manually first before triggering retraction
- **move_base not found**: Ensure manual_exploration.launch is running
- **Retraction timeout**: Normal behavior if path is long or obstacles block route

## Check Recorded Path
```bash
rosparam get /path_recorder_node/path_length
```

That's it! Simple manual exploration with automatic path recording.
