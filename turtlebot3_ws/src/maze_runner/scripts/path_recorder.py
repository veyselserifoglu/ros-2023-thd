#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class PathRecorder:
    def __init__(self):
        rospy.init_node('path_recorder_node', anonymous=True)
        
        # Parameters for recording thresholds
        self.distance_threshold = 0.5  # meters
        self.angle_threshold = math.radians(20)  # degrees
        
        # Storage for recorded path
        self.recorded_path = []
        self.last_recorded_pose = None
        self.last_recorded_yaw = None
        
        # Subscribe to odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        rospy.loginfo("Path recorder initialized. Recording path with thresholds: %.1fm, %.0f°", 
                     self.distance_threshold, math.degrees(self.angle_threshold))
    
    def odom_callback(self, msg):
        current_pose = msg.pose.pose
        
        # Extract position and orientation
        x = current_pose.position.x
        y = current_pose.position.y
        
        # Convert quaternion to yaw angle
        orientation_q = current_pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
        # Check if we should record this pose
        if self.should_record_pose(x, y, yaw):
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = current_pose
            
            self.recorded_path.append(pose_stamped)
            self.last_recorded_pose = (x, y)
            self.last_recorded_yaw = yaw
            
            # Store in ROS parameter for other nodes to access
            self.update_path_parameter()
            
            rospy.loginfo("Recorded pose #%d: (%.2f, %.2f, %.1f°)", 
                         len(self.recorded_path), x, y, math.degrees(yaw))
    
    def should_record_pose(self, x, y, yaw):
        if self.last_recorded_pose is None:
            return True
        
        # Calculate distance from last recorded pose
        dx = x - self.last_recorded_pose[0]
        dy = y - self.last_recorded_pose[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate angle difference
        angle_diff = abs(yaw - self.last_recorded_yaw)
        # Normalize angle difference to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        angle_diff = abs(angle_diff)
        
        # Record if either threshold is exceeded
        return distance >= self.distance_threshold or angle_diff >= self.angle_threshold
    
    def update_path_parameter(self):
        # Convert path to parameter format
        path_data = []
        for pose_stamped in self.recorded_path:
            pose = pose_stamped.pose
            path_data.append({
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'qx': pose.orientation.x,
                'qy': pose.orientation.y,
                'qz': pose.orientation.z,
                'qw': pose.orientation.w
            })
        
        # Store in ROS parameter server
        rospy.set_param('/path_recorder_node/recorded_path', path_data)
        rospy.set_param('/path_recorder_node/path_length', len(path_data))

if __name__ == '__main__':
    try:
        recorder = PathRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Path recorder shutting down")
