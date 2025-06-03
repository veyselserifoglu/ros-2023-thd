# TurtleBot3 Maze Navigation System

A ROS-based autonomous navigation system that allows manual maze exploration with intelligent path recording and autonomous retraction using backtracking algorithms.

## 📋 System Overview

This system provides a complete maze navigation solution with two main phases:
1. **Manual Exploration**: Drive the robot manually while automatically recording the path
2. **Autonomous Retraction**: Robot autonomously navigates back to start using recorded waypoints

## 🚀 Quick Start Commands

### **Step 1: Launch Manual Exploration**
```bash
cd /turtlebot3_ws
source devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch maze_runner manual_exploration.launch
```
*This will automatically open a teleop window for manual control*

*To record some path positions, explore the map manually before running the recording & retracting:*

## 🎮 Manual Control Keys
- `w` - Move forward
- `a` - Turn left  
- `s` - Stop
- `d` - Turn right
- `x` - Move backward

### **Step 2: Start A* Retractor (New Terminal)**
```bash
cd /turtlebot3_ws
source devel/setup.bash
export TURTLEBOT3_MODEL=burger
python3 /turtlebot3_ws/src/maze_runner/scripts/simple_astar_retractor.py
```

### **Step 3: Trigger Autonomous Retraction (New Terminal)**
```bash
cd /turtlebot3_ws
source devel/setup.bash
rosservice call /simple_astar_retractor/trigger_retraction "{}"
```

## 📍 Path Recorder - Technical Details

### **How Path Recording Works**
The path recorder intelligently captures robot positions during manual exploration using smart filtering to avoid cluttered waypoints.

#### **Core Mechanism**
- **Timer-based polling**: Checks robot position every 0.3 seconds
- **TF transform system**: Uses `/map` → `/base_link` transformation for precise positioning
- **Smart filtering**: Only records waypoints that meet specific criteria

#### **Recording Algorithm**
```python
# Position acquisition
(trans, rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
x, y = trans[0], trans[1]  # Robot's X,Y coordinates in map frame
yaw = tf.transformations.euler_from_quaternion(rot)[2]  # Robot's heading

# Distance calculation
distance_moved = sqrt((x - last_x)² + (y - last_y)²)

# Angle change calculation  
angle_diff = abs(current_yaw - last_yaw)
if angle_diff > π: angle_diff = 2π - angle_diff  # Handle wraparound

# Recording criteria (ALL must be met)
significant_movement = distance_moved ≥ 0.8m  # Moved at least 80cm
significant_rotation = angle_diff ≥ 0.4 rad   # Turned at least 23°
time_constraint = time_since_last ≥ 1.0s      # At least 1 second gap

# Record waypoint if: (movement OR rotation) AND time_constraint
should_record = (significant_movement OR significant_rotation) AND time_constraint
```

#### **Waypoint Data Structure**
```python
waypoint = {
    'x': 2.45,           # X position in meters
    'y': -1.23,          # Y position in meters  
    'yaw': 1.57,         # Heading in radians
    'timestamp': 1234567890.12  # Unix timestamp
}
```

#### **Why This Prevents "Rapid W-Key" Problems**
- **Distance threshold**: Ignores micro-movements < 80cm
- **Time constraint**: Prevents spam recording from rapid key presses
- **Result**: Clean path with 10-20 meaningful waypoints instead of hundreds

## 🤖 Simple A* Retractor - Technical Details

### **How Autonomous Retraction Works**
The A* retractor uses recorded waypoints to navigate back to the starting position with intelligent backtracking when stuck.

#### **Core Components**
- **ROS Service**: Waits for `/simple_astar_retractor/trigger_retraction` calls
- **Action Client**: Sends navigation goals to `move_base` 
- **TF Listener**: Gets robot's current position for distance calculations

#### **Waypoint Selection Algorithm**
```python
# Adaptive selection based on path length
if path_length ≤ 8:    # Short: use all waypoints
    waypoints_to_use = recorded_waypoints[::-1]
elif path_length ≤ 16: # Medium: every 2nd waypoint  
    waypoints_to_use = recorded_waypoints[::2][::-1]
else:                  # Long: distance-based (≥2m apart)
    selected = distance_based_filtering(recorded_waypoints)
    waypoints_to_use = selected[::-1]
```

#### **Navigation Strategy**
- **Normal goal**: 40s timeout, 0.8m position tolerance
- **Proximity skip**: If already within 1.2m of target waypoint
- **Failure handling**: After 2 failures → trigger backtracking algorithm

#### **Backtracking Algorithm**
```python
# When robot gets stuck, try going back to previous waypoints
for steps_back in [1, 2, 3]:  # Progressive backtracking
    backtrack_waypoint = waypoints[current_idx - steps_back]
    distance_to_backtrack = calculate_distance(current_pos, backtrack_waypoint)
    
    if 0.5m ≤ distance_to_backtrack ≤ 4.0m:  # Valid backtrack range
        navigate_with_relaxed_parameters(backtrack_waypoint, timeout=25s)
        if success: 
            resume_from_backtrack_position()
            break
```

## 🔄 Complete Navigation Flow Diagram

```
Service Call → trigger_retraction()
     ↓
📂 Get recorded waypoints from parameter server
     ↓
🎯 Smart waypoint selection (short/medium/long logic)
     ↓
🔄 Reverse waypoint order (end → start)
     ↓
🚀 For each waypoint:
     ├── 📏 Check proximity (< 1.2m) → Skip if close
     ├── 🎯 Send navigation goal to move_base
     ├── ⏱️ Wait for result (40s timeout)
     ├── ✅ Success → Move to next waypoint
     └── ❌ Failure → Increment failure count
         └── 🔄 If 2+ failures → BACKTRACKING
             ├── 🔙 Try 1, 2, 3 waypoints back
             ├── 📏 Validate distance (0.5m-4.0m)
             ├── 🎯 Navigate with relaxed parameters (25s)
             └── ✅ Success → Resume from backtrack position
     ↓
📊 Return service response: success/failure + completion stats
```

## 🛠️ Complete System Components

### **Directory Structure**
```
maze_runner/
├── launch/           # ROS launch files
├── scripts/          # Python executable scripts  
├── param/            # Configuration files
├── worlds/           # Gazebo simulation environments
├── src/              # C++ source files (if any)
├── CMakeLists.txt    # Build configuration
├── package.xml       # ROS package metadata
└── README.md         # This documentation
```

### **Launch Files**
- **`manual_exploration.launch`** - Main system launcher
  - Starts Gazebo with maze world
  - Launches TurtleBot3 robot in simulation
  - Initializes SLAM (Gmapping) for real-time mapping
  - Starts navigation stack (move_base) with optimized parameters
  - Automatically launches path recorder
  - Opens teleop window for manual control
  
- **`smooth_move_base.launch`** - Navigation stack configuration
  - Optimized DWA local planner parameters for maze navigation
  - Global planner configuration for autonomous navigation
  - Costmap settings for obstacle avoidance
  - Recovery behavior configuration

### **Scripts**
- **`path_recorder.py`** - Intelligent waypoint recording system
  - Timer-based position monitoring (every 0.3s)
  - Smart filtering: 0.8m distance OR 0.4 rad rotation threshold
  - TF transform-based precise positioning
  - ROS parameter server storage for persistence
  
- **`simple_astar_retractor.py`** - Autonomous retraction service
  - ROS service interface for triggering retraction
  - Adaptive waypoint selection for different path lengths
  - Progressive backtracking algorithm for obstacle recovery
  - Move_base action client for autonomous navigation

### **Parameter Files**
- **`smooth_dwa_local_planner_params.yaml`** - DWA planner configuration
  ```yaml
  # Velocity limits optimized for maze navigation
  max_vel_x: 0.35          # Maximum forward velocity
  max_vel_theta: 3.5       # Maximum angular velocity
  
  # Goal tolerance for successful navigation
  xy_goal_tolerance: 0.8   # Position tolerance (80cm)
  yaw_goal_tolerance: 1.0  # Orientation tolerance (~57°)
  
  # Trajectory scoring for path following
  path_distance_bias: 32.0 # Favor following recorded path
  goal_distance_bias: 24.0 # Balance between path following and goal reaching
  occdist_scale: 0.05      # Obstacle avoidance sensitivity
  ```

- **`maze_exploration.rviz`** - RViz visualization configuration
  - Robot model display for visual feedback
  - Real-time map visualization from SLAM
  - Laser scan data overlay
  - Path visualization for recorded/planned routes
  - Goal markers and navigation feedback
  - Optimized layout for maze exploration monitoring

### **World Files**
- **`simple_box_maze.world`** - Gazebo simulation environment
  ```xml
  <!-- 7x7 meter maze with strategic layout -->
  <world name='large_maze_world'>
    <!-- Basic lighting and ground plane -->
    <!-- Outer perimeter walls (0.1m thick, 1.0m high) -->
    <!-- Internal maze structure with dead ends -->
    <!-- South entrance at (0, -3.5) for robot entry -->
    <!-- Various corridors and turns for navigation testing -->
  ```
  
  **Maze Features:**
  - **Size**: 7m × 7m navigable area
  - **Wall thickness**: 0.1m for realistic collision detection
  - **Wall height**: 1.0m for laser scan visibility
  - **Entry point**: South side entrance at coordinates (0, -3.5)
  - **Complexity**: Multiple dead ends and branching paths
  - **Testing scenarios**: Narrow corridors, sharp turns, backtracking opportunities

### **Build Files**
- **`CMakeLists.txt`** - Catkin build configuration
  - Package dependencies (rospy, actionlib, tf, geometry_msgs)
  - Python script installation paths
  - Launch file installation directories
  
- **`package.xml`** - ROS package metadata
  - Package information and dependencies
  - Build and runtime requirements
  - Maintainer and license information

## 🔧 Useful Commands

### **Monitor recorded path:**
```bash
rosparam get /path_recorder_node/recorded_path
```

### **Check system status:**
```bash
rosnode list                    # See running nodes
rosservice list | grep retraction  # Check retraction service
rostopic list | grep move_base     # Verify navigation topics
```

### **Debug navigation:**
```bash
rostopic echo /move_base/goal       # Monitor navigation goals
rostopic echo /move_base/result     # Monitor navigation results
```

## 🎯 Key Features

### **Path Recorder**
- **Intelligent filtering**: Prevents waypoint spam from rapid movements
- **Adaptive thresholds**: 0.8m distance OR 23° rotation triggers
- **Persistent storage**: Saves to ROS parameter server for cross-node access

### **A* Retractor**
- **Adaptive waypoint selection**: Handles short/long paths differently
- **Smart proximity detection**: Skips waypoints when already close
- **Robust backtracking**: Recovers from navigation failures
- **Safety limits**: Prevents infinite loops with attempt counters

### **Navigation System**
- **Maze-optimized parameters**: Tuned for narrow corridors and tight turns
- **Dynamic parameter adjustment**: Relaxed tolerances for difficult waypoints
- **Real-time feedback**: Detailed logging of navigation progress