#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Twist, Pose, PoseArray
from sensor_msgs.msg import LaserScan

class BoidSeparationNode:

    def __init__(self):
        # Create subcribers to receive info from pose and laser
        rospy.Subscriber('/ground_truth_pose', Pose, self.pose_callback)
        rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)

        # Create a publisher to publish velocity
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Set boid parameters
        self.max_linear_speed = 0.2
        self.max_angular_speed = 1.0
        self.separation_distance = 0.5
    
    def pose_callback(self, data):
        # Calculate separation vector to avoid collision between boids
        separation_vector = self.calculate_separation_vector(data)

        # Calculate the parameters for velocity
        angle = atan2(separation_vector[1], separation_vector[0])
        magnitude = min(sqrt(separation_vector[0]**2 + separation_vector[1]**2), self.max_linear_speed)

        # Create a Twist message to publish velocity
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = magnitude * cos(angle)
        cmd_vel_msg.linear.y = magnitude * sin(angle)

        self.cmd_vel_pub.publish(cmd_vel_msg)

    def laser_callback(self, data):
        if data.ranges:
            separation_vector = self.calculate_separation_vector(data)
        else:
            rospy.logwarn("No data received from laser scan")
    
    # Calculate the separation vector based on the Reynolds rule
    def calculate_separation_vector(self, laser_data):
        pos = [] # Store boid positions
        separation_vector = [0.0, 0.0]

        # Iterate through laser scan ranges
        for i, range_value in enumerate(laser_data.ranges):
            if laser_data.range_min < range_value < laser_data.range_max:
                angle = laser_data.angle_min + i * laser_data.angle_increment

                # Claculate the position of the detected point
                x = laser_data.position.x + range_value * cos(angle)
                y = laser_data.position.y + range_value * sin(angle)

                pos.append((x, y)) # Add new boid position
        
        for actual_boid_pos in pos:
            dx = laser_data.position.x - actual_boid_pos[0]
            dy = laser_data.position.y - actual_boid_pos[1]

            distance = sqrt(dx**2 + dy**2)

            # Separation rule
            if distance < self.separation_distance:
                separation_vector[0] += dx / distance
                separation_vector[1] += dy / distance
        
        return separation_vector

if __name__ == '__main__':
    rospy.init_node('boid_separation_node', anonymous=True)
    node = BoidSeparationNode()
    rospy.spin()