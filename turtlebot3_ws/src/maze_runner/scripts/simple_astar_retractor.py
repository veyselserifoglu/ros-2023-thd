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
        
        rospy.loginfo("Simple A* Retractor initialized with backtracking capability")
        
        # Wait for move_base
        if self.move_base_client.wait_for_server(timeout=rospy.Duration(30.0)):
            rospy.loginfo("SUCCESS: Connected to move_base action server")
        else:
            rospy.logerr("FAILED: move_base action server not available")

    def get_current_position(self):
        """Get current robot position"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            return trans[0], trans[1]
        except:
            return None, None

    def calculate_distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def navigate_to_waypoint(self, x, y, timeout=45.0):
        """Navigate to a specific waypoint"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        
        rospy.loginfo("Attempting to reach waypoint: ({:.2f}, {:.2f})".format(x, y))
        self.move_base_client.send_goal(goal)
        
        return self.move_base_client.wait_for_result(rospy.Duration(timeout))

    def backtrack_and_retry(self, waypoints, current_idx, failed_attempts):
        """
        Improved backtracking strategy: Go back to a validated previous waypoint
        """
        if current_idx <= 0:
            rospy.logwarn("Cannot backtrack further - at first waypoint")
            return False, current_idx
        
        current_x, current_y = self.get_current_position()
        if current_x is None:
            return False, current_idx
        
        # Try multiple backtrack distances to find a suitable target
        for backtrack_steps in [1, 2, 3]:
            backtrack_idx = max(0, current_idx - backtrack_steps)
            
            if backtrack_idx == current_idx:
                continue  # Skip if same as current
                
            backtrack_waypoint = waypoints[backtrack_idx]
            
            # Check if backtrack target is reasonable
            distance_to_backtrack = self.calculate_distance(
                current_x, current_y, 
                backtrack_waypoint['x'], backtrack_waypoint['y']
            )
            
            # Validate backtrack distance (must be meaningful but not too far)
            if distance_to_backtrack < 0.5:  # Too close, no point
                rospy.loginfo("Backtrack target {} too close ({:.2f}m), trying further back".format(
                    backtrack_idx + 1, distance_to_backtrack))
                continue
                
            if distance_to_backtrack > 4.0:  # Too far, might fail too
                rospy.loginfo("Backtrack target {} too far ({:.2f}m), trying closer".format(
                    backtrack_idx + 1, distance_to_backtrack))
                continue
            
            # This is a good backtrack target
            rospy.loginfo("BACKTRACKING: Going back {} steps to waypoint {} (distance: {:.2f}m)".format(
                backtrack_steps, backtrack_idx + 1, distance_to_backtrack))
            
            # Set more permissive parameters for backtracking
            rospy.set_param("/move_base/xy_goal_tolerance", 1.2)
            rospy.set_param("/move_base/yaw_goal_tolerance", 2.0)
            rospy.set_param("/move_base/DWAPlannerROS/max_vel_x", 0.3)  # Slower for safety
            
            # Try to reach backtrack waypoint with shorter timeout
            success = self.navigate_to_waypoint(
                backtrack_waypoint['x'], 
                backtrack_waypoint['y'], 
                timeout=25.0  # Shorter timeout for backtrack
            )
            
            if success:
                rospy.loginfo("Backtrack successful! Now retrying from waypoint {}".format(backtrack_idx + 1))
                
                # Reset navigation parameters
                rospy.set_param("/move_base/xy_goal_tolerance", 0.8)
                rospy.set_param("/move_base/yaw_goal_tolerance", 1.0)
                rospy.set_param("/move_base/DWAPlannerROS/max_vel_x", 0.4)
                
                return True, backtrack_idx  # Resume from backtrack position
            else:
                rospy.logwarn("Backtrack to waypoint {} failed, trying different target".format(backtrack_idx + 1))
        
        # All backtrack attempts failed
        rospy.logwarn("All backtrack attempts failed, skipping current waypoint")
        return False, current_idx

    def trigger_retraction(self, req):
        """Start retraction using recorded waypoints with backtracking"""
        rospy.loginfo("*** Retraction with backtracking triggered! ***")
        
        # Get recorded path
        if not rospy.has_param("/path_recorder_node/recorded_path"):
            return TriggerResponse(False, "No recorded path found. Explore first!")
        
        try:
            # Set optimized navigation parameters
            rospy.set_param("/move_base/xy_goal_tolerance", 0.8)
            rospy.set_param("/move_base/yaw_goal_tolerance", 1.0)
            rospy.set_param("/move_base/DWAPlannerROS/max_vel_x", 0.4)
            rospy.set_param("/move_base/DWAPlannerROS/min_vel_x", -0.2)
            rospy.set_param("/move_base/DWAPlannerROS/max_vel_trans", 0.4)
            rospy.set_param("/move_base/DWAPlannerROS/min_vel_trans", 0.1)
            
            recorded_waypoints = rospy.get_param("/path_recorder_node/recorded_path")
            if not recorded_waypoints or len(recorded_waypoints) < 2:
                return TriggerResponse(False, "Recorded path too short")
            
            rospy.loginfo("Found {} recorded waypoints".format(len(recorded_waypoints)))
            
            # Smart waypoint selection (improved)
            path_length = len(recorded_waypoints)
            
            if path_length <= 8:
                # Short path: use all waypoints
                waypoints_to_use = recorded_waypoints[::-1]
                rospy.loginfo("Short path: using all {} waypoints".format(len(waypoints_to_use)))
            elif path_length <= 16:
                # Medium path: every 2nd waypoint
                waypoints_to_use = recorded_waypoints[::2][::-1]
                rospy.loginfo("Medium path: using every 2nd waypoint ({} total)".format(len(waypoints_to_use)))
            else:
                # Long path: distance-based selection
                selected = [recorded_waypoints[0]]  # Always include start
                last_selected = recorded_waypoints[0]
                
                for waypoint in recorded_waypoints[1:]:
                    distance = self.calculate_distance(
                        last_selected['x'], last_selected['y'],
                        waypoint['x'], waypoint['y']
                    )
                    if distance >= 2.0:  # At least 2m apart
                        selected.append(waypoint)
                        last_selected = waypoint
                
                # Ensure we include the end (which becomes start after reversal)
                if selected[-1] != recorded_waypoints[-1]:
                    selected.append(recorded_waypoints[-1])
                
                waypoints_to_use = selected[::-1]
                rospy.loginfo("Long path: distance-based selection ({} total)".format(len(waypoints_to_use)))
            
            # Limit total waypoints to prevent complexity
            if len(waypoints_to_use) > 10:
                # Keep first, last, and evenly distribute middle waypoints
                step = len(waypoints_to_use) // 8
                waypoints_to_use = [waypoints_to_use[i] for i in range(0, len(waypoints_to_use), step)]
                waypoints_to_use.append(recorded_waypoints[0])  # Ensure end at start
                rospy.loginfo("Reduced to {} key waypoints".format(len(waypoints_to_use)))
            
            # Navigation with backtracking
            successful_waypoints = 0
            failed_attempts = {}  # Track failed attempts per waypoint
            current_waypoint_idx = 0
            max_total_attempts = len(waypoints_to_use) * 3  # Prevent infinite loops
            total_attempts = 0
            
            while current_waypoint_idx < len(waypoints_to_use) and total_attempts < max_total_attempts:
                waypoint = waypoints_to_use[current_waypoint_idx]
                total_attempts += 1
                
                # Initialize failed attempts counter for this waypoint
                if current_waypoint_idx not in failed_attempts:
                    failed_attempts[current_waypoint_idx] = 0
                
                # Check if already close to this waypoint
                current_x, current_y = self.get_current_position()
                if current_x is not None:
                    distance_to_waypoint = self.calculate_distance(
                        current_x, current_y, waypoint['x'], waypoint['y']
                    )
                    
                    if distance_to_waypoint < 1.2:  # Close enough
                        rospy.loginfo("Already close to waypoint {}/{} ({:.2f}m), moving to next".format(
                            current_waypoint_idx + 1, len(waypoints_to_use), distance_to_waypoint))
                        successful_waypoints += 1
                        current_waypoint_idx += 1
                        continue
                
                rospy.loginfo("Navigating to waypoint {}/{}: ({:.2f}, {:.2f})".format(
                    current_waypoint_idx + 1, len(waypoints_to_use), waypoint['x'], waypoint['y']))
                
                # Try to navigate to waypoint
                success = self.navigate_to_waypoint(waypoint['x'], waypoint['y'], timeout=40.0)
                
                if success:
                    rospy.loginfo("SUCCESS: Reached waypoint {}/{}".format(
                        current_waypoint_idx + 1, len(waypoints_to_use)))
                    successful_waypoints += 1
                    current_waypoint_idx += 1
                    failed_attempts[current_waypoint_idx] = 0  # Reset failure count
                else:
                    failed_attempts[current_waypoint_idx] += 1
                    rospy.logwarn("FAILED: Could not reach waypoint {}/{} (attempt {})".format(
                        current_waypoint_idx + 1, len(waypoints_to_use), 
                        failed_attempts[current_waypoint_idx]))
                    
                    # Try backtracking if we've failed multiple times
                    if failed_attempts[current_waypoint_idx] >= 2:
                        rospy.loginfo("Multiple failures detected, attempting backtrack...")
                        backtrack_success, new_idx = self.backtrack_and_retry(
                            waypoints_to_use, current_waypoint_idx, failed_attempts
                        )
                        
                        if backtrack_success:
                            current_waypoint_idx = new_idx  # Resume from backtrack position
                            failed_attempts[current_waypoint_idx] = 0  # Reset failure count
                        else:
                            # Backtrack failed, skip this waypoint
                            rospy.logwarn("Backtrack failed, skipping waypoint {}".format(
                                current_waypoint_idx + 1))
                            current_waypoint_idx += 1
                    else:
                        # First failure, just retry next iteration
                        rospy.loginfo("Retrying waypoint {} (attempt {})".format(
                            current_waypoint_idx + 1, failed_attempts[current_waypoint_idx] + 1))
            
            # Final result
            if total_attempts >= max_total_attempts:
                return TriggerResponse(False, "Maximum attempts reached, stopping to prevent infinite loop")
            
            if successful_waypoints >= len(waypoints_to_use) * 0.7:  # 70% success rate
                return TriggerResponse(True, "Retraction completed! Reached {}/{} waypoints".format(
                    successful_waypoints, len(waypoints_to_use)))
            else:
                return TriggerResponse(False, "Retraction partially failed. Reached only {}/{} waypoints".format(
                    successful_waypoints, len(waypoints_to_use)))
                
        except Exception as e:
            rospy.logerr("Retraction error: {}".format(str(e)))
            return TriggerResponse(False, "Retraction failed: {}".format(str(e)))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        retractor = SimpleAStarRetractor()
        retractor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("A* Retractor shutdown")