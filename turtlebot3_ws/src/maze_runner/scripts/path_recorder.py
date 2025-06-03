#!/usr/bin/env python3
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped

class PathRecorder:
    def __init__(self):
        rospy.init_node('path_recorder_node', anonymous=True)
        
        # More balanced parameters for better coverage
        self.min_distance = rospy.get_param("~min_distance", 0.8)  # Reduced to 0.8m
        self.min_angle_change = rospy.get_param("~min_angle_change", 0.4)  # Reduced to 0.4 rad (~23 degrees)
        
        # TF listener for getting robot position
        self.tf_listener = tf.TransformListener()
        
        # Recorded path storage
        self.recorded_path = []
        self.last_recorded_pose = None
        self.last_recorded_yaw = None
        self.last_check_time = None
        
        # More frequent checking for better responsiveness
        self.recording_timer = rospy.Timer(rospy.Duration(0.3), self.record_position)
        
        rospy.loginfo("Path Recorder started - balanced waypoint recording")
        rospy.loginfo("Min distance: {}m, Min angle change: {} rad".format(self.min_distance, self.min_angle_change))

    def get_current_pose_and_yaw(self):
        """Get current robot pose and yaw angle"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            return trans[0], trans[1], yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None, None, None

    def should_record_waypoint(self, x, y, yaw):
        """Determine if current position should be recorded as waypoint"""
        if self.last_recorded_pose is None:
            return True
        
        # Calculate distance moved
        dx = x - self.last_recorded_pose[0]
        dy = y - self.last_recorded_pose[1]
        distance_moved = math.sqrt(dx*dx + dy*dy)
        
        # Calculate angle change
        angle_diff = abs(yaw - self.last_recorded_yaw)
        if angle_diff > math.pi:
            angle_diff = 2*math.pi - angle_diff
        
        # Record only if significant movement OR significant rotation
        significant_movement = distance_moved >= self.min_distance
        significant_rotation = angle_diff >= self.min_angle_change
        
        # Reduced time constraint to 1 second (was 2 seconds)
        time_check = True
        if self.last_check_time is not None:
            time_since_last = rospy.Time.now() - self.last_check_time
            time_check = time_since_last.to_sec() >= 1.0
        
        should_record = (significant_movement or significant_rotation) and time_check
        
        if should_record:
            rospy.loginfo("Recording waypoint: moved {:.2f}m, turned {:.2f} rad".format(
                distance_moved, angle_diff))
        
        return should_record

    def record_position(self, event):
        """Record current position if it meets the criteria"""
        x, y, yaw = self.get_current_pose_and_yaw()
        
        if x is None:
            return
        
        if self.should_record_waypoint(x, y, yaw):
            # Record this waypoint
            waypoint = {
                'x': x,
                'y': y,
                'yaw': yaw,
                'timestamp': rospy.Time.now().to_sec()
            }
            
            self.recorded_path.append(waypoint)
            self.last_recorded_pose = (x, y)
            self.last_recorded_yaw = yaw
            self.last_check_time = rospy.Time.now()
            
            # Save to parameter server
            rospy.set_param("/path_recorder_node/recorded_path", self.recorded_path)
            
            rospy.loginfo("Waypoint {}: ({:.2f}, {:.2f}) - Total waypoints: {}".format(
                len(self.recorded_path), x, y, len(self.recorded_path)))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        recorder = PathRecorder()
        recorder.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Path Recorder shutdown")