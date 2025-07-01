#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseArray
from std_msgs.msg import String
from nav_msgs.msg import Path

class Visualization:
    def __init__(self):
        rospy.init_node('visualization', anonymous=True)
        
        # Publishers
        self.exploration_path_pub = rospy.Publisher('/exploration_path', Marker, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/rrt_samples', PoseArray, self.rrt_samples_callback)
        rospy.Subscriber('/exploration_state', String, self.state_callback)
        
        # Tracked data
        self.exploration_path = []
        self.current_state = "INIT"
        
        # Set update rate
        self.rate = rospy.Rate(1)  # 1 Hz for visualization updates
        
        rospy.loginfo("Visualization node initialized")
        
    def rrt_samples_callback(self, pose_array):
        """Process RRT sample points to visualize exploration path"""
        if pose_array.poses:
            # Add the latest pose to our exploration path
            latest_pose = pose_array.poses[-1]
            point = (latest_pose.position.x, latest_pose.position.y)
            
            # Avoid duplicates
            if not self.exploration_path or point != self.exploration_path[-1]:
                self.exploration_path.append(point)
                self.publish_exploration_path()
    
    def state_callback(self, msg):
        """Track the current state of the exploration"""
        self.current_state = msg.data
        rospy.loginfo("Current state: %s", self.current_state)
    
    def publish_exploration_path(self):
        """Publish the robot's exploration path as a line strip marker"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "exploration_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03  # Line width
        
        # Color based on state
        if self.current_state == "EXPLORATION":
            # Blue during exploration
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.8
        elif self.current_state == "RETURN_TO_START":
            # Green during return
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
        else:
            # Default purple
            marker.color.r = 0.5
            marker.color.g = 0.0
            marker.color.b = 0.5
            marker.color.a = 0.8
        
        # Add points to the line strip
        for x, y in self.exploration_path:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.05
            marker.points.append(point)
        
        self.exploration_path_pub.publish(marker)
    
    def run(self):
        """Main loop"""
        while not rospy.is_shutdown():
            # Update visualization periodically
            if self.exploration_path:
                self.publish_exploration_path()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        visualization = Visualization()
        visualization.run()
    except rospy.ROSInterruptException:
        pass
