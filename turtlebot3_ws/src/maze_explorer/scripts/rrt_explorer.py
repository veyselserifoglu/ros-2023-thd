#!/usr/bin/env python3

import rospy
import math
import numpy as np
import random
from geometry_msgs.msg import PoseArray, Pose, Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler
# --- KEY ADDITION: Import actionlib and move_base messages ---
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf

class RRTExplorer:
    def __init__(self):
        rospy.init_node('rrt_explorer', anonymous=True)
        
        # Parameters
        self.target_x = rospy.get_param('~target_x', 4.0)
        self.target_y = rospy.get_param('~target_y', -4.0)
        self.step_size = rospy.get_param('~step_size', 0.3)
        self.goal_bias = rospy.get_param('~goal_bias', 0.2)
        self.max_iterations = rospy.get_param('~max_iterations', 5000)
        self.target_threshold = rospy.get_param('~target_threshold', 0.5)
        
        self.initialized = False
        
        # RRT data structures
        self.rrt_tree = []
        self.rrt_edges = []
        self.current_position = None
        
        # Map data
        self.map_data = None
        self.map_metadata = None
        
        # Publishers
        self.samples_pub = rospy.Publisher('/rrt_samples', PoseArray, queue_size=1)
        self.path_viz_pub = rospy.Publisher('/rrt_path', Marker, queue_size=1)
        self.target_reached_pub = rospy.Publisher('/target_reached', Bool, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Action Client for move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base action server found.")

        self.tf_listener = tf.TransformListener()
        
        
        rospy.loginfo("RRT Explorer initialized")
    
    def odom_callback(self, data):
        self.current_position = (data.pose.pose.position.x, data.pose.pose.position.y)
        if not self.initialized and self.current_position:
            self.rrt_tree.append(self.current_position)
            self.initialized = True
            rospy.loginfo("RRT tree initialized at: %.2f, %.2f", self.current_position[0], self.current_position[1])

    # --- KEY ADDITION: Function to send a goal to move_base ---
    def send_goal_to_move_base(self, point):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = point[0]
        goal.target_pose.pose.position.y = point[1]
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(f"Sending goal: ({point[0]:.2f}, {point[1]:.2f})")
        self.move_base_client.send_goal(goal)
        
        # Wait for the server to finish performing the action.
        self.move_base_client.wait_for_result()
        
        # Return the state of the action
        return self.move_base_client.get_state()

    # --- KEY MODIFICATION: The main RRT loop is now the main execution flow ---
    def run(self):
        rospy.loginfo("Waiting for initial position...")
        while not self.initialized and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # --- KEY CHANGE: A more robust wait ---
        rospy.loginfo("Waiting for the map to odom transform to be available...")
        try:
            # This will wait until the transform is publishing, for up to 10 seconds.
            self.tf_listener.waitForTransform("map", "odom", rospy.Time(0), rospy.Duration(10.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logerr("Could not get transform from map to odom! Shutting down.")
            return

        rospy.loginfo("Transform is available! Starting RRT exploration.")
        iteration = 0
        target_reached = False
        
        # ... The rest of the run() method (the 'while' loop) stays exactly the same as before ...
        while not rospy.is_shutdown() and iteration < self.max_iterations and not target_reached:
            random_point = self.sample_point()
            nearest_idx = self.nearest_node(random_point)
            nearest_node = self.rrt_tree[nearest_idx]
            new_point = self.steer(nearest_node, random_point)
            
            if self.is_collision_free(nearest_node, new_point):
                rospy.loginfo(f"--- RRT Iteration {iteration}: Attempting to move to new node. ---")
                action_state = self.send_goal_to_move_base(new_point)
                
                if action_state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Move successful! Adding node to the RRT tree.")
                    self.rrt_tree.append(new_point)
                    self.rrt_edges.append((nearest_idx, len(self.rrt_tree) - 1))
                    self.publish_rrt_visualization()
                    
                    if self.euclidean_distance(new_point, (self.target_x, self.target_y)) < self.target_threshold:
                        rospy.loginfo("Target reached by RRT!")
                        self.target_reached_pub.publish(Bool(data=True))
                        target_reached = True
                else:
                    rospy.logwarn(f"Move to new node failed with state: {action_state}. RRT will try a different direction.")
            iteration += 1
        
        if not target_reached:
            rospy.logwarn("RRT exploration finished without reaching the target.")

    # (Keep all your other helper methods: map_callback, sample_point, steer, is_collision_free, etc.)
    def map_callback(self, data):
        self.map_data = data.data
        self.map_metadata = data.info

    def euclidean_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def sample_point(self):
        if random.random() < self.goal_bias:
            return (self.target_x, self.target_y)
        else:
            if self.map_metadata:
                width = self.map_metadata.width * self.map_metadata.resolution
                height = self.map_metadata.height * self.map_metadata.resolution
                origin_x = self.map_metadata.origin.position.x
                origin_y = self.map_metadata.origin.position.y
                return (random.uniform(origin_x, origin_x + width), random.uniform(origin_y, origin_y + height))
            else:
                return (random.uniform(-5.0, 5.0), random.uniform(-5.0, 5.0))

    def nearest_node(self, point):
        distances = [self.euclidean_distance(point, node) for node in self.rrt_tree]
        return np.argmin(distances)

    def steer(self, from_point, to_point):
        dist = self.euclidean_distance(from_point, to_point)
        if dist <= self.step_size:
            return to_point
        else:
            theta = math.atan2(to_point[1] - from_point[1], to_point[0] - from_point[0])
            return (from_point[0] + self.step_size * math.cos(theta),
                   from_point[1] + self.step_size * math.sin(theta))

    def world_to_map(self, x, y):
        if not self.map_metadata: return None
        map_x = int((x - self.map_metadata.origin.position.x) / self.map_metadata.resolution)
        map_y = int((y - self.map_metadata.origin.position.y) / self.map_metadata.resolution)
        if (0 <= map_x < self.map_metadata.width and 0 <= map_y < self.map_metadata.height):
            return (map_x, map_y)
        return None

    def is_collision_free(self, from_point, to_point):
        if not self.map_data: return True
        dist = self.euclidean_distance(from_point, to_point)
        num_points = int(dist / self.map_metadata.resolution)
        for i in range(num_points + 1):
            t = float(i) / float(num_points)
            x = from_point[0] * (1 - t) + to_point[0] * t
            y = from_point[1] * (1 - t) + to_point[1] * t
            map_coords = self.world_to_map(x, y)
            if not map_coords: return False
            idx = map_coords[1] * self.map_metadata.width + map_coords[0]
            if self.map_data[idx] > 50: return False
        return True

    def publish_rrt_visualization(self):
        # (Your visualization code remains the same)
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = rospy.Time.now()
        for point in self.rrt_tree:
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose_array.poses.append(pose)
        self.samples_pub.publish(pose_array)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "rrt_edges"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.a = 0.8
        marker.color.g = 0.8
        marker.color.b = 0.8
        for parent_idx, child_idx in self.rrt_edges:
            p1 = self.rrt_tree[parent_idx]
            p2 = self.rrt_tree[child_idx]
            marker.points.append(Point(x=p1[0], y=p1[1]))
            marker.points.append(Point(x=p2[0], y=p2[1]))
        self.path_viz_pub.publish(marker)

if __name__ == '__main__':
    try:
        explorer = RRTExplorer()
        # --- KEY CHANGE: Call the main run loop ---
        explorer.run()
    except rospy.ROSInterruptException:
        pass