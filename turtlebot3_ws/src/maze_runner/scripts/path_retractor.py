#!/usr/bin/env python
import rospy
import actionlib
import math
import threading
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import tf.transformations

class PathRetractor:
    def __init__(self):
        rospy.init_node('path_retractor_node', anonymous=True)
        self.lock = threading.Lock()  # Thread lock for thread safety
        self.active_retraction = False  # Flag to track if retraction is active
        self.abort_requested = False   # Flag to allow aborting current retraction
        
        # Parameters
        self.recorded_path_param = "/path_recorder_node/recorded_path"
        self.simplify_path = rospy.get_param("~simplify_path", True)
        self.goal_timeout = rospy.get_param("~goal_timeout", 45.0)  # Increased timeout for key waypoints
        self.max_consecutive_failures = rospy.get_param("~max_consecutive_failures", 2)
        self.xy_goal_tolerance = rospy.get_param("~xy_goal_tolerance", 0.2)  # More lenient tolerance
        
        # Smart path summarization parameters
        self.min_keypoint_distance = rospy.get_param("~min_keypoint_distance", 1.0)  # Minimum 1m between key points
        self.angle_threshold = rospy.get_param("~angle_threshold", 30.0)  # 30 degrees for direction changes
        self.max_keypoints = rospy.get_param("~max_keypoints", 15)  # Maximum number of key waypoints
        
        # Action client for move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        try:
            server_up = self.move_base_client.wait_for_server(timeout=rospy.Duration(10.0))
            if not server_up:
                rospy.logerr("Timed out waiting for move_base action server")
                return
            rospy.loginfo("Connected to move_base action server.")
        except rospy.ROSException as e:
            rospy.logerr("Failed to connect to move_base action server: %s" % e)
            return
        
        # TF listener for getting current position
        self.tf_listener = tf.TransformListener()
        
        # Publishers for visualization
        self.path_pub = rospy.Publisher('~retraction_path', Path, queue_size=1, latch=True)
        self.markers_pub = rospy.Publisher('~waypoint_markers', MarkerArray, queue_size=1, latch=True)
        
        # Services
        self.trigger_service = rospy.Service('~trigger_retraction', Empty, self.handle_trigger_retraction)
        self.abort_service = rospy.Service('~abort_retraction', Trigger, self.handle_abort_retraction)
        self.status_service = rospy.Service('~retraction_status', Trigger, self.handle_status_request)

        rospy.loginfo("PathRetractor node ready. Call the 'trigger_retraction' service to start.")

    def handle_trigger_retraction(self, req):
        with self.lock:
            if self.active_retraction:
                rospy.logwarn("Retraction already in progress. Ignoring new request.")
                return EmptyResponse()
            
            self.active_retraction = True
            self.abort_requested = False
        
        # Start retraction in a separate thread to keep the service responsive
        threading.Thread(target=self.execute_path_retraction).start()
        return EmptyResponse()

    def handle_abort_retraction(self, req):
        with self.lock:
            if not self.active_retraction:
                return TriggerResponse(success=False, message="No active retraction to abort")
            
            self.abort_requested = True
            self.move_base_client.cancel_all_goals()
            return TriggerResponse(success=True, message="Abort requested")

    def handle_status_request(self, req):
        with self.lock:
            if self.active_retraction:
                msg = "Retraction in progress"
                if self.abort_requested:
                    msg += " (aborting)"
                return TriggerResponse(success=True, message=msg)
            return TriggerResponse(success=True, message="No active retraction")

    def get_current_pose(self):
        try:
            self.tf_listener.waitForTransform("map", "base_footprint", rospy.Time(0), rospy.Duration(5.0))
            (trans, rot) = self.tf_listener.lookupTransform("map", "base_footprint", rospy.Time(0))
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position = Point(trans[0], trans[1], trans[2])
            pose.pose.orientation = Quaternion(rot[0], rot[1], rot[2], rot[3])
            return pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Error: %s" % e)
            return None

    def extract_key_waypoints(self, path_data):
        """
        Extract key waypoints from the recorded path using intelligent analysis.
        This creates a much shorter list of strategic waypoints instead of following every recorded point.
        """
        if not path_data or len(path_data) < 2:
            return path_data
            
        rospy.loginfo("Analyzing path to extract key waypoints...")
        
        key_points = []
        
        # Always include the starting point
        key_points.append(path_data[0])
        rospy.loginfo("Added start point: ({:.2f}, {:.2f})".format(path_data[0][0], path_data[0][1]))
        
        # For large paths, use more aggressive sampling
        if len(path_data) > 500:
            # For very large paths, sample every N points and then filter
            sample_factor = max(10, len(path_data) // 100)
            sampled_path = [path_data[i] for i in range(0, len(path_data), sample_factor)]
            rospy.loginfo("Pre-sampling large path: {} -> {} points".format(len(path_data), len(sampled_path)))
        else:
            sampled_path = path_data
            
        # Analyze for significant direction changes and distance-based waypoints
        last_direction = None
        last_keypoint = path_data[0]
        
        for i in range(1, len(sampled_path) - 1):
            current_point = sampled_path[i]
            next_point = sampled_path[i + 1]
            
            # Calculate distance from last keypoint
            dist_from_last = math.sqrt(
                (current_point[0] - last_keypoint[0])**2 + 
                (current_point[1] - last_keypoint[1])**2
            )
            
            # Calculate current direction
            dx = next_point[0] - current_point[0]
            dy = next_point[1] - current_point[1]
            current_direction = math.atan2(dy, dx)
            
            # Decide if this should be a keypoint
            should_add = False
            reason = ""
            
            # Add point if we've traveled minimum distance
            if dist_from_last >= self.min_keypoint_distance:
                should_add = True
                reason = "distance ({:.2f}m)".format(dist_from_last)
            
            # Add point if direction changed significantly
            elif last_direction is not None:
                angle_diff = abs(current_direction - last_direction)
                # Handle angle wraparound
                if angle_diff > math.pi:
                    angle_diff = 2 * math.pi - angle_diff
                    
                if math.degrees(angle_diff) > self.angle_threshold:
                    should_add = True
                    reason = "direction change ({:.1f}°)".format(math.degrees(angle_diff))
            
            if should_add:
                key_points.append(current_point)
                rospy.loginfo("Added keypoint {}: ({:.2f}, {:.2f}) - {}".format(
                    len(key_points), current_point[0], current_point[1], reason))
                last_keypoint = current_point
                last_direction = current_direction
                
                # Safety check - don't add too many keypoints
                if len(key_points) >= self.max_keypoints - 1:  # -1 to leave room for endpoint
                    rospy.logwarn("Reached maximum keypoints limit ({}), stopping analysis".format(self.max_keypoints))
                    break
            else:
                # Update direction even if we don't add the point
                last_direction = current_direction
        
        # Always include the final point
        if len(path_data) > 1:
            final_point = path_data[-1]
            # Only add if it's not too close to the last keypoint
            if len(key_points) > 0:
                dist_to_final = math.sqrt(
                    (final_point[0] - key_points[-1][0])**2 + 
                    (final_point[1] - key_points[-1][1])**2
                )
                if dist_to_final > 0.3:  # At least 30cm from last keypoint
                    key_points.append(final_point)
                    rospy.loginfo("Added end point: ({:.2f}, {:.2f})".format(final_point[0], final_point[1]))
                else:
                    rospy.loginfo("End point too close to last keypoint, skipping")
            else:
                key_points.append(final_point)
        
        rospy.loginfo("Path summarization complete: {} -> {} keypoints".format(
            len(path_data), len(key_points)))
        
        return key_points

    def get_recorded_path(self):
        try:
            path_data = rospy.get_param(self.recorded_path_param)
            if not path_data:
                rospy.logwarn("Recorded path is empty or not found.")
                return None
            return path_data
        except KeyError:
            rospy.logerr("ROS Parameter '{}' not found.".format(self.recorded_path_param))
            return None
        except Exception as e:
            rospy.logerr("Error retrieving path from parameter server: {}".format(e))
            return None

    def create_path_markers(self, path_points):
        marker_array = MarkerArray()
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        
        for i, pose_array in enumerate(path_points):
            # Create path message
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = rospy.Time.now()
            ps.pose.position = Point(pose_array[0], pose_array[1], pose_array[2])
            ps.pose.orientation = Quaternion(pose_array[3], pose_array[4], pose_array[5], pose_array[6])
            path_msg.poses.append(ps)
            
            # Create marker for this waypoint
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(pose_array[0], pose_array[1], pose_array[2])
            marker.pose.orientation = Quaternion(0, 0, 0, 1)
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0 if i == 0 else 0.0
            marker.color.g = 0.0 if i == 0 else 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)
            
            # Add a line connecting waypoints
            if i > 0:
                line = Marker()
                line.header.frame_id = "map"
                line.header.stamp = rospy.Time.now()
                line.ns = "waypoint_connections"
                line.id = i + 1000  # Offset to avoid ID collision
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                
                # Start point
                p1 = Point()
                p1.x = path_points[i-1][0]
                p1.y = path_points[i-1][1]
                p1.z = path_points[i-1][2]
                
                # End point
                p2 = Point()
                p2.x = pose_array[0]
                p2.y = pose_array[1]
                p2.z = pose_array[2]
                
                line.points.append(p1)
                line.points.append(p2)
                line.scale.x = 0.05
                line.color.r = 0.0
                line.color.g = 0.8
                line.color.b = 0.8
                line.color.a = 0.8
                line.lifetime = rospy.Duration(0)
                marker_array.markers.append(line)
        
        # Publish the path and markers
        self.path_pub.publish(path_msg)
        self.markers_pub.publish(marker_array)

    def execute_path_retraction(self):
        try:
            rospy.loginfo("Starting intelligent path retraction...")
            
            path_data = self.get_recorded_path()
            if not path_data:
                rospy.loginfo("No path to retract.")
                with self.lock:
                    self.active_retraction = False
                return

            rospy.loginfo("Retrieved recorded path with {} points.".format(len(path_data)))
            
            # Reverse the path for retraction (go back to start)
            reversed_path = list(reversed(path_data))
            
            # Extract key waypoints using intelligent analysis
            key_waypoints = self.extract_key_waypoints(reversed_path)
            
            if not key_waypoints:
                rospy.logerr("No key waypoints extracted from path!")
                with self.lock:
                    self.active_retraction = False
                return
            
            # Create and publish visualization markers
            self.create_path_markers(key_waypoints)
            
            # Get current position to decide starting point
            current_pose = self.get_current_pose()
            start_idx = 0
            
            if current_pose and len(key_waypoints) > 1:
                # Skip waypoints that are very close to current position
                for i, waypoint in enumerate(key_waypoints):
                    dist_to_waypoint = math.sqrt(
                        (current_pose.pose.position.x - waypoint[0])**2 + 
                        (current_pose.pose.position.y - waypoint[1])**2
                    )
                    if dist_to_waypoint > 0.5:  # At least 0.5m away
                        start_idx = i
                        break
                    else:
                        rospy.loginfo("Skipping keypoint {}: too close ({:.2f}m)".format(i+1, dist_to_waypoint))
            
            # Execute navigation to each key waypoint
            success_count = 0
            total_keypoints = len(key_waypoints) - start_idx
            
            rospy.loginfo("Navigating through {} key waypoints starting from index {}".format(
                total_keypoints, start_idx + 1))
            
            for i in range(start_idx, len(key_waypoints)):
                # Check for abort request
                with self.lock:
                    if self.abort_requested:
                        rospy.loginfo("Aborting path retraction as requested.")
                        break
                
                waypoint = key_waypoints[i]
                
                # Create the goal
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position = Point(waypoint[0], waypoint[1], waypoint[2])
                
                # Set orientation - point toward next waypoint or use recorded orientation for final point
                if i < len(key_waypoints) - 1:
                    next_waypoint = key_waypoints[i + 1]
                    dx = next_waypoint[0] - waypoint[0]
                    dy = next_waypoint[1] - waypoint[1]
                    yaw = math.atan2(dy, dx)
                    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
                    goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                else:
                    # Final waypoint - use recorded orientation
                    goal.target_pose.pose.orientation = Quaternion(waypoint[3], waypoint[4], waypoint[5], waypoint[6])
                
                # Log the goal
                rospy.loginfo("Navigating to keypoint {}/{}: ({:.2f}, {:.2f})".format(
                    i - start_idx + 1, total_keypoints, waypoint[0], waypoint[1]))
                
                # Send goal to move_base
                self.move_base_client.send_goal(goal)
                
                # Wait for result with timeout
                finished_within_time = self.move_base_client.wait_for_result(
                    rospy.Duration(self.goal_timeout))
                
                if not finished_within_time:
                    self.move_base_client.cancel_goal()
                    rospy.logwarn("Timeout reaching keypoint {}/{}. Continuing to next keypoint.".format(
                        i - start_idx + 1, total_keypoints))
                else:
                    state = self.move_base_client.get_state()
                    if state == actionlib.GoalStatus.SUCCEEDED:
                        rospy.loginfo("Successfully reached keypoint {}/{}".format(
                            i - start_idx + 1, total_keypoints))
                        success_count += 1
                    else:
                        rospy.logwarn("Failed to reach keypoint {}/{}: {}".format(
                            i - start_idx + 1, total_keypoints, self.move_base_client.get_goal_status_text()))
                
                # Small delay between waypoints to let the robot stabilize
                rospy.sleep(1.0)
            
            # Final summary
            rospy.loginfo("Path retraction completed! Successfully reached {} out of {} key waypoints.".format(
                success_count, total_keypoints))
            
            if success_count == total_keypoints:
                rospy.loginfo("🎉 Perfect retraction! Robot successfully returned to start.")
            elif success_count > total_keypoints * 0.7:  # More than 70% success
                rospy.loginfo("✅ Good retraction! Robot made significant progress back to start.")
            else:
                rospy.logwarn("⚠️  Partial retraction. Robot may need manual assistance.")
                
        except Exception as e:
            rospy.logerr("Exception during path retraction: %s" % e)
        finally:
            with self.lock:
                self.active_retraction = False
                self.abort_requested = False

if __name__ == '__main__':
    try:
        retractor = PathRetractor()
        rospy.spin()  # Keep the node alive
    except rospy.ROSInterruptException:
        rospy.loginfo("PathRetractor node terminated.")
