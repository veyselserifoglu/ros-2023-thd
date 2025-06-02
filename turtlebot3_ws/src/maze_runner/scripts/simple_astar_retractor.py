#!/usr/bin/env python3
import rospy
import actionlib
import tf
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse

class SimpleAStarRetractor:
    def __init__(self):
        rospy.init_node('simple_astar_retractor', anonymous=False)
        
        # TF listener
        self.tf_listener = tf.TransformListener()
        
        # Move base action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # Service
        self.trigger_service = rospy.Service('~trigger_retraction', Trigger, self.trigger_retraction)
        
        rospy.loginfo("Simple A* Retractor initialized")
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait for move_base with longer timeout and better error handling
        rospy.loginfo("Checking if action topics exist...")
        try:
            # Check if action topics exist
            topics = rospy.get_published_topics()
            action_topics = [topic for topic, _ in topics if 'move_base' in topic and any(x in topic for x in ['goal', 'result', 'status'])]
            rospy.loginfo("Found move_base action topics: {}".format(action_topics))
            
            rospy.loginfo("Waiting for action server (30 second timeout)...")
            if self.move_base_client.wait_for_server(timeout=rospy.Duration(30.0)):
                rospy.loginfo("SUCCESS: Connected to move_base action server")
            else:
                rospy.logerr("FAILED: move_base action server not available after 30 seconds")
                rospy.logerr("This may indicate a configuration issue with move_base")
        except Exception as e:
            rospy.logerr("Error connecting to move_base: {}".format(e))

    def get_current_position(self):
        """Get current robot position"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            return trans[0], trans[1]
        except:
            return None, None

    def trigger_retraction(self, req):
        """Start retraction using recorded waypoints"""
        rospy.loginfo("*** Retraction triggered! ***")
        
        # Get recorded path
        if not rospy.has_param("/path_recorder_node/recorded_path"):
            return TriggerResponse(False, "No recorded path found. Explore first!")
        
        try:
            recorded_waypoints = rospy.get_param("/path_recorder_node/recorded_path")
            if not recorded_waypoints or len(recorded_waypoints) < 2:
                return TriggerResponse(False, "Recorded path too short")
            
            # Get start position (first recorded waypoint)
            start_pos = recorded_waypoints[0]
            
            rospy.loginfo("Navigating back to start position: ({:.2f}, {:.2f})".format(start_pos['x'], start_pos['y']))
            
            # Create goal for start position
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = start_pos['x']
            goal.target_pose.pose.position.y = start_pos['y']
            goal.target_pose.pose.orientation.w = 1.0
            
            # Send goal
            self.move_base_client.send_goal(goal)
            
            # Wait for result
            rospy.loginfo("Moving to start position...")
            if self.move_base_client.wait_for_result(rospy.Duration(60.0)):
                result = self.move_base_client.get_state()
                if result == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("*** Retraction completed successfully! ***")
                    return TriggerResponse(True, "Successfully returned to start position")
                else:
                    rospy.logwarn("Failed to reach start position")
                    return TriggerResponse(False, "Failed to reach start position")
            else:
                rospy.logwarn("Timeout reaching start position")
                self.move_base_client.cancel_goal()
                return TriggerResponse(False, "Timeout reaching start position")
            
        except Exception as e:
            rospy.logerr("Retraction failed: {}".format(e))
            return TriggerResponse(False, "Error: {}".format(e))

if __name__ == '__main__':
    try:
        retractor = SimpleAStarRetractor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
