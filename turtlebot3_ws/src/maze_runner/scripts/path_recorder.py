#!/usr/bin/env python3
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped

class PathRecorder:
    def __init__(self):
        rospy.init_node('path_recorder_node', anonymous=True)
        
        # Parameters
        self.min_distance = rospy.get_param("~min_distance", 0.5)  # Minimum distance between waypoints
        self.min_angle_change = rospy.get_param("~min_angle_change", 0.3)  # Minimum angle change (radians)
        
        # TF listener for getting robot position
        self.tf_listener = tf.TransformListener()
        
        # Recorded path storage
        self.recorded_path = []
        self.last_recorded_pose = None
        self.last_recorded_yaw = None
        
        # Timer for recording
        self.recording_timer = rospy.Timer(rospy.Duration(0.2), self.record_position)
        
        rospy.loginfo("Path Recorder started - recording waypoints every 0.2s")
        rospy.loginfo("Min distance: {}m, Min angle change: {} rad".format(self.min_distance, self.min_angle_change))

    def get_current_pose_and_yaw(self):
        """Get current robot pose and yaw angle"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            
            # Convert quaternion to yaw
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            
            return trans[0], trans[1], yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None, None, None

    def should_record_waypoint(self, x, y, yaw):
        """Determine if current position should be recorded as waypoint"""
        if self.last_recorded_pose is None:
            return True
        
        # Calculate distance from last recorded position
        dx = x - self.last_recorded_pose[0]
        dy = y - self.last_recorded_pose[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate angle change
        angle_change = abs(yaw - self.last_recorded_yaw)
        if angle_change > math.pi:
            angle_change = 2*math.pi - angle_change
        
        # Record if moved far enough or turned significantly
        return distance >= self.min_distance or angle_change >= self.min_angle_change

    def record_position(self, event):
        """Record current position if criteria met"""
        x, y, yaw = self.get_current_pose_and_yaw()
        if x is None:
            return
        
        if self.should_record_waypoint(x, y, yaw):
            waypoint = {
                'x': x,
                'y': y,
                'yaw': yaw,
                'timestamp': rospy.Time.now().to_sec()
            }
            
            self.recorded_path.append(waypoint)
            self.last_recorded_pose = (x, y)
            self.last_recorded_yaw = yaw
            
            # Store on parameter server for retractor to access
            rospy.set_param("/path_recorder_node/recorded_path", self.recorded_path)
            
            rospy.loginfo("Recorded waypoint #{}: ({:.2f}, {:.2f}, {:.2f})".format(len(self.recorded_path), x, y, yaw))

    def get_recorded_path(self):
        """Get the full recorded path"""
        return self.recorded_path

if __name__ == '__main__':
    try:
        recorder = PathRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
