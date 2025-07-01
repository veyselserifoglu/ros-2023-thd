#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler

class WaypointManager:
    def __init__(self):
        rospy.init_node('waypoint_manager', anonymous=True)
        
        # Parameters
        self.min_waypoint_distance = 0.5  # Minimum distance between waypoints
        self.grid_size = 0.2  # Grid cell size for waypoint optimization
        
        # Initialize waypoints storage
        self.waypoints = []  # List of (x, y) waypoints
        self.grid_waypoints = {}  # Dictionary mapping grid cells to waypoints
        
        # Publishers
        self.waypoints_marker_pub = rospy.Publisher('/waypoints', Marker, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/add_waypoint', Pose, self.add_waypoint_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        self.map_data = None
        self.map_metadata = None
        
        # Set update rate
        self.rate = rospy.Rate(1)  # 1 Hz for visualization updates
        
        rospy.loginfo("Waypoint Manager initialized")
    
    def map_callback(self, data):
        """Store map data for path planning"""
        self.map_data = data.data
        self.map_metadata = data.info
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid cell coordinates"""
        grid_x = int(x / self.grid_size)
        grid_y = int(y / self.grid_size)
        return (grid_x, grid_y)
    
    def is_junction(self, x, y):
        """Detect if a point is at a junction by checking surrounding cells"""
        if not self.map_data or not self.map_metadata:
            return False
        
        # Convert world coordinates to map coordinates
        map_x = int((x - self.map_metadata.origin.position.x) / self.map_metadata.resolution)
        map_y = int((y - self.map_metadata.origin.position.y) / self.map_metadata.resolution)
        
        # Check if point is valid in map
        if (map_x < 0 or map_x >= self.map_metadata.width or
            map_y < 0 or map_y >= self.map_metadata.height):
            return False
        
        # Check for open adjacent cells in four directions (N, E, S, W)
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        open_directions = 0
        
        for dx, dy in directions:
            nx, ny = map_x + dx, map_y + dy
            if (0 <= nx < self.map_metadata.width and 
                0 <= ny < self.map_metadata.height):
                idx = ny * self.map_metadata.width + nx
                if 0 <= idx < len(self.map_data) and self.map_data[idx] < 50:
                    open_directions += 1
        
        # Return true if there are more than 2 open directions (indicating a junction)
        return open_directions > 2
    
    def add_waypoint_callback(self, pose):
        """Process new waypoints, perform optimization and store them"""
        x, y = pose.position.x, pose.position.y
        
        # Check if we should add this waypoint
        add_waypoint = True
        
        # Skip if too close to an existing waypoint
        for wp_x, wp_y in self.waypoints:
            dist = np.sqrt((x - wp_x)**2 + (y - wp_y)**2)
            if dist < self.min_waypoint_distance:
                add_waypoint = False
                break
        
        # Use grid-based waypoint storage for optimization
        grid_cell = self.world_to_grid(x, y)
        if grid_cell in self.grid_waypoints:
            add_waypoint = False
        
        # Check if it's a junction (prioritize junctions for waypoints)
        is_junction_point = self.is_junction(x, y)
        
        if add_waypoint or is_junction_point:
            rospy.loginfo("Adding waypoint at: %.2f, %.2f (junction: %s)", 
                          x, y, "yes" if is_junction_point else "no")
            
            self.waypoints.append((x, y))
            self.grid_waypoints[grid_cell] = (x, y)
            
            # Update visualization
            self.publish_waypoints_marker()
    
    def publish_waypoints_marker(self):
        """Publish visualization markers for waypoints"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        for x, y in self.waypoints:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.1
            marker.points.append(point)
        
        self.waypoints_marker_pub.publish(marker)
    
    def run(self):
        """Main loop"""
        while not rospy.is_shutdown():
            # Update waypoint visualization periodically
            self.publish_waypoints_marker()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        waypoint_manager = WaypointManager()
        waypoint_manager.run()
    except rospy.ROSInterruptException:
        pass
