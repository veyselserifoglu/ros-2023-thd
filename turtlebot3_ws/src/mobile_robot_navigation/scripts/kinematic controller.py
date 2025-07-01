# check the controler.py, change the x_pos to ge the current robotic positions. #!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import numpy as np
import math

class TurtleBot3Controller:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('turtlebot3_controller', anonymous=True)

        # Initialize position and orientation variables
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.yaw = 0.0

        # Goal position
        self.x_goal = 2.0
        self.y_goal = 2.0

        # Controller gains
        self.K_l = 0.5
        self.k_a = 5
        self.k_b = -3.5

        # Subscriber to the /odom topic
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publisher to the /cmd_vel topic
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Set publishing rate
        self.rate = rospy.Rate(10)  # 10 Hz

    def odom_callback(self, msg):
        # Extract position and orientation from the Odometry message
        position = msg.pose.pose.position
        orientation_quat = msg.pose.pose.orientation
        self.x_pos = position.x
        self.y_pos = position.y

        # Convert quaternion to Euler angles
        orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = yaw

    def compute_control(self):


        # Current position and orientation (observation)
        x_start = self.x_pos
        y_start = self.y_pos
        rotation = self.yaw

        # Goal position (0, 0)
        goal_x, goal_y = self.x_goal, self.y_goal
        goal_z = 0

        # Compute errors
        error_x = goal_x - x_start
        error_y = goal_y - y_start
        error_rot_origin = goal_z - rotation

        error_rot_frame = -rotation + np.arctan2(error_y, error_x)
        beta = -rotation - error_rot_frame

        # Distance to goal
        dist2goal = math.sqrt(error_x ** 2 + error_y ** 2)

        # Control laws for linear and angular velocity
        v = self.K_l * dist2goal
        w = self.k_a * error_rot_frame + self.k_b * beta

        # Handle reversing direction when the goal is behind the robot
        if -np.pi < error_rot_frame <= -np.pi/2 or np.pi/2 < error_rot_frame <= np.pi:
            v = -v

        return v, w

    def run(self):
        while not rospy.is_shutdown():
            # Compute control inputs
            linear_vel, angular_vel = self.compute_control()

            # Create a Twist message
            velocity = Twist()
            velocity.linear.x = linear_vel
            velocity.angular.z = angular_vel

            # Publish the velocity command
            self.cmd_pub.publish(velocity)

            # Sleep to maintain the loop rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Create the controller object and run it
        controller = TurtleBot3Controller()
        controller.run()
    except rospy.ROSInterruptException:
        pass
