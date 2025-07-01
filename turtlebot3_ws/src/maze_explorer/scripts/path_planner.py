#!/usr/bin/env python3

import rospy
import numpy as np
import heapq
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner', anonymous=True)
        
        # Parameters
        self.start_x = rospy.get_param('~start_x', -4.0)
        self.start_y = rospy.get_param('~start_y', 4.0)
        self.target_x = rospy.get_param('~target_x', 4.0)
        self.target_y = rospy.get_param('~target_y', -4.0)
        
        # Map data
        self.map_data = None
        self.map_metadata = None
        self.waypoints = []
        
        # Publishers
        self.path_pub = rospy.Publisher('/optimal_path', Path, queue_size=1)
        self.return_path_pub = rospy.Publisher('/return_path', Marker, queue_size=1)
        self.path_ready_pub = rospy.Publisher('/path_planning_complete', Bool, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/waypoints', Marker, self.waypoints_callback)
        rospy.Subscriber('/plan_return_path', Bool, self.plan_path_callback)
        
        rospy.loginfo("Path Planner initialized")
    
    def map_callback(self, data):
        """Store map data for path planning"""
        self.map_data = data.data
        self.map_metadata = data.info
        rospy.loginfo("Map received with resolution: %f", self.map_metadata.resolution)
    
    def waypoints_callback(self, marker):
        """Process waypoints from waypoint manager"""
        self.waypoints = [(point.x, point.y) for point in marker.points]
        rospy.loginfo("Received %d waypoints", len(self.waypoints))
    
    def plan_path_callback(self, msg):
        """Trigger path planning when requested"""
        if msg.data and self.map_data and self.waypoints:
            rospy.loginfo("Planning return path...")
            self.plan_return_path()
    
    def euclidean_distance(self, p1, p2):
        """Calculate Euclidean distance between two points"""
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid cell coordinates"""
        grid_x = int((x - self.map_metadata.origin.position.x) / self.map_metadata.resolution)
        grid_y = int((y - self.map_metadata.origin.position.y) / self.map_metadata.resolution)
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid cell coordinates to world coordinates"""
        x = grid_x * self.map_metadata.resolution + self.map_metadata.origin.position.x
        y = grid_y * self.map_metadata.resolution + self.map_metadata.origin.position.y
        return (x, y)
    
    def is_valid_point(self, grid_x, grid_y):
        """Check if a grid point is valid (within bounds and not an obstacle)"""
        if (grid_x < 0 or grid_x >= self.map_metadata.width or
            grid_y < 0 or grid_y >= self.map_metadata.height):
            return False
        
        idx = grid_y * self.map_metadata.width + grid_x
        if 0 <= idx < len(self.map_data):
            # Consider unknown space (-1) as free for exploration
            return self.map_data[idx] < 50
        return False
    
    def get_neighbors(self, grid_point):
        """Get valid neighboring grid points"""
        grid_x, grid_y = grid_point
        
        # 8-connected grid: consider all adjacent cells including diagonals
        directions = [
            (0, 1),    # North
            (1, 1),    # Northeast
            (1, 0),    # East
            (1, -1),   # Southeast
            (0, -1),   # South
            (-1, -1),  # Southwest
            (-1, 0),   # West
            (-1, 1)    # Northwest
        ]
        
        neighbors = []
        for dx, dy in directions:
            new_x, new_y = grid_x + dx, grid_y + dy
            if self.is_valid_point(new_x, new_y):
                neighbors.append((new_x, new_y))
        
        return neighbors
    
    def heuristic(self, grid_point, goal_grid_point):
        """A* heuristic function - Euclidean distance"""
        return self.euclidean_distance(
            self.grid_to_world(grid_point[0], grid_point[1]),
            self.grid_to_world(goal_grid_point[0], goal_grid_point[1])
        )
    
    def a_star(self, start_point, goal_point):
        """A* algorithm implementation for path planning"""
        if not self.map_data:
            rospy.logwarn("No map data available for path planning")
            return None
        
        # Convert world coordinates to grid coordinates
        start_grid = self.world_to_grid(start_point[0], start_point[1])
        goal_grid = self.world_to_grid(goal_point[0], goal_point[1])
        
        rospy.loginfo("Planning path from grid cell %s to %s", str(start_grid), str(goal_grid))
        
        # Initialize A* data structures
        open_set = []  # Priority queue of (f_score, grid_point)
        heapq.heappush(open_set, (0, start_grid))
        
        came_from = {}  # Dictionary to reconstruct path
        g_score = {start_grid: 0}  # Cost from start to current node
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}  # Estimated total cost
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal_grid:
                # Reconstruct and return the path
                path = []
                while current in came_from:
                    world_point = self.grid_to_world(current[0], current[1])
                    path.append(world_point)
                    current = came_from[current]
                
                # Add the start point
                world_point = self.grid_to_world(start_grid[0], start_grid[1])
                path.append(world_point)
                
                # Path is from goal to start, so reverse it
                path.reverse()
                return path
            
            for neighbor in self.get_neighbors(current):
                # Calculate the tentative g_score
                # Use Euclidean distance for diagonal movement cost
                world_current = self.grid_to_world(current[0], current[1])
                world_neighbor = self.grid_to_world(neighbor[0], neighbor[1])
                
                tentative_g_score = g_score[current] + self.euclidean_distance(world_current, world_neighbor)
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # This path to neighbor is better than any previous one
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score_value = tentative_g_score + self.heuristic(neighbor, goal_grid)
                    f_score[neighbor] = f_score_value
                    
                    # Add to open set if not there
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score_value, neighbor))
        
        rospy.logwarn("No path found!")
        return None
    
    def plan_return_path(self):
        """Plan the return path from target to start"""
        if not self.map_data:
            rospy.logwarn("Waiting for map data...")
            return
        
        start_point = (self.target_x, self.target_y)  # Start from target (point B)
        goal_point = (self.start_x, self.start_y)     # Return to start (point A)
        
        rospy.loginfo("Planning path from (%.2f, %.2f) to (%.2f, %.2f)",
                     start_point[0], start_point[1], goal_point[0], goal_point[1])
        
        # Run A* algorithm
        path = self.a_star(start_point, goal_point)
        
        if path:
            rospy.loginfo("Path found with %d points", len(path))
            self.publish_path(path)
            self.path_ready_pub.publish(Bool(data=True))
        else:
            rospy.logwarn("Failed to find a path")
            self.path_ready_pub.publish(Bool(data=False))
    
    def publish_path(self, path):
        """Publish the planned path for visualization and execution"""
        # Create ROS Path message
        ros_path = Path()
        ros_path.header.frame_id = "map"
        ros_path.header.stamp = rospy.Time.now()
        
        for point in path:
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)
        
        self.path_pub.publish(ros_path)
        
        # Create a visualization marker for the path
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "return_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  # Line width
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        for x, y in path:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.1
            marker.points.append(point)
        
        self.return_path_pub.publish(marker)

if __name__ == '__main__':
    try:
        path_planner = PathPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
