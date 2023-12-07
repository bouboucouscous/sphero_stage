#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import re
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from obstacle_avoidance import ObstacleAvoidance
import time

import json
import os

class Alignment:

    def __init__(self):
        self.num_of_robots = rospy.get_param("/num_of_robots")
        # create a list and a flag indicating if the target position has been received
        self.target_received = False
        self.target_position = np.array([0.0, 0.0])

        # create a list indicating if each robot has reached the target
        self.target_reached = [False for i in range(self.num_of_robots)]

        # Parameters for field of view
        self.neighbours_dist = 1.0
        # self.neighbours_angle = 2*np.pi/3
        self.neighbours_angle = np.pi
        
        # separation distance to be maintained
        self.separation_dist = 0.4

        #max force of cohesion
        self.max_force = 10

        # max and min velocities for the robots
        self.max_linear_velx = 0.4
        self.min_linear_velx = -0.4
        self.max_linear_vely = 0.4
        self.min_linear_vely = -0.4

        # velocity vector weights for each behaviour
        self.align_vel_weights = 0.8
        self.cohesion_vel_weights = 0.09
        self.separation_vel_weights = 2.0
        self.steer_vel_weight = 0.10
        self.obs_vel_weight = 9.5

        # listen to the transforms between robots to compute their relative transformations
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # arrays to store robot postions and velocities
        self.positions = np.zeros((self.num_of_robots, 2))
        self.velocities = np.zeros((self.num_of_robots, 2))

        # arrays to store the velocities for each of the behaviours
        self.align_vel = np.zeros((self.num_of_robots, 2))
        self.cohesion_vel = np.zeros((self.num_of_robots, 2))
        self.separation_vel = np.zeros((self.num_of_robots, 2))
        self.steer_vel = np.zeros((self.num_of_robots, 2))
        self.obs_vel = np.zeros((self.num_of_robots, 2))

        # data saving
        self.file_path = "/home/mawais/catkin_ws/src/sphero_simulation/sphero_stage/resources/data/velocities.json"
        if os.path.exists(self.file_path):
            os.remove(self.file_path)
            print("File has been deleted.")
        else:
            print("File does not exist.")

        # create an instance of the obstacle avoidance class
        self.obstacle = ObstacleAvoidance()

        # publishers to publish weighted velocities to each robot
        self.cmd_vel_pub = [rospy.Publisher("/robot_{}/cmd_vel".format(i), Twist, queue_size=1)
                                for i in range(self.num_of_robots)]
        
        
        # subscribers to get the positions and velocities of each robot
        [rospy.Subscriber("/robot_{}/odom".format(i), Odometry, self.odom_callback)
          for i in range(self.num_of_robots)]
        
        # subscriber to get the target position from rviz
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.target_position_callback)

        # timer to call the weighted_velocities function
        rospy.Timer(rospy.Duration(0.05), self.weighted_velocities)
        

    def odom_callback(self, odom):
        # Define a regular expression pattern to match the number
        pattern = re.compile(r"/robot_(\d+)/odom")
        frame_id = odom.header.frame_id
        # Use the pattern to search for a match in the string
        match = pattern.search(frame_id)
        robot_num = int(match.group(1))
        # Store position and velocity of all robots
        p = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y])
        self.positions[robot_num] = p
        v = np.array([odom.twist.twist.linear.x, odom.twist.twist.linear.y])
        self.velocities[robot_num] = v


    def target_position_callback(self, goal):
        self.target_position = np.array([goal.pose.position.x, goal.pose.position.y])
        self.target_received = True

    def neighbours(self, robot_num):
        # Find neighbours of robot_num
        neighbours = []
        for i in range(len(self.positions)):
            if i != robot_num:
                try:
                    # Get the transform from robot_num to robot i
                    transform = self.tfBuffer.lookup_transform("robot_{}/base_footprint".format(robot_num),
                                                                "robot_{}/base_footprint".format(i), rospy.Time(0),
                                                                  rospy.Duration(1.0))
                    # print("Transform from 'source_frame' to 'target_frame':", transform.transform.translation.x, transform.transform.translation.y)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    print("Error:", e)
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                distance = np.sqrt(x**2 + y**2)
                theta = np.arctan2(y, x)
                if distance < self.neighbours_dist and (theta <= self.neighbours_angle
                                                         and theta >= -self.neighbours_angle):
                    # print(x,y,theta)
                    neighbours.append(i)
        return neighbours
    
    # def test_neighbours(self, event):
    #     for i in range(self.num_of_robots):
    #         neighbours = self.neighbours(i)
    #         print("Robot {} has neighbours {}".format(i, neighbours))
    
    def average_velocity(self, neighbours):
        # Find average velocity of neighbours of robot_num
        average_velocity = np.zeros(2)
        for i in neighbours:
            average_velocity += self.velocities[i]
        average_velocity /= len(neighbours)
        return average_velocity
    
    def align(self):
        # find the average velocity in the neighborhood and align with it
        for i in range(self.num_of_robots):
            neighbours = self.neighbours(i)
            if len(neighbours) != 0:
                average_velocity = self.average_velocity(neighbours)
                current_velocity = average_velocity
                vx = max(self.min_linear_velx, min(current_velocity[0], self.max_linear_velx))
                vy = max(self.min_linear_vely, min(current_velocity[1], self.max_linear_vely))
                v = np.array([vx, vy])
                self.align_vel[i] = v

    def average_position(self, robot_num):
        # Find average position of neighbours of robot_num
        neighbours = self.neighbours(robot_num)
        average_position = np.zeros(2)
        for i in neighbours:
            average_position += self.positions[i]
        average_position /= max(1, len(neighbours))
        return average_position
    
    def cohesion(self):
        # find the average position in the neighborhood and move towards it
        for i in range(self.num_of_robots):
            neighbours = self.neighbours(i)
            if len(neighbours) != 0:
                average_position = self.average_position(i)
                current_position = self.positions[i]
                difference = average_position - current_position
                distance = np.linalg.norm(difference)  # Calcul mean dist
                normalized_difference = difference / (distance + 1e-5)  # Normalization to avoid division by zero
                magnitude = min(distance, self.max_force)  # Adjust this according to your needs
                v = magnitude * normalized_difference
                self.cohesion_vel[i] = v

    def separation(self):
        # find the neighbours in the neighborhood and move away from them
        for i in range(self.num_of_robots):
            neighbours = self.neighbours(i)
            force = np.zeros(2)
            for neighbour in neighbours:
                difference = self.positions[i] - self.positions[neighbour]
                if np.linalg.norm(difference) < self.separation_dist:
                    force += difference/((np.linalg.norm(difference)+0.00001))
            
            force /= max(1, len(neighbours))
            vx = max(self.min_linear_velx, min(force[0], self.max_linear_velx))
            vy = max(self.min_linear_vely, min(force[1], self.max_linear_vely))
            v = np.array([vx, vy])
            self.separation_vel[i] = v

    def steer(self):
        # steer towards the target position
        if self.target_received:
            for i in range(self.num_of_robots):
                current_position = self.positions[i]
                difference = self.target_position - current_position
                vx = max(self.min_linear_velx, min(difference[0], self.max_linear_velx))
                vy = max(self.min_linear_vely, min(difference[1], self.max_linear_vely))
                v = np.array([vx, vy])
                if np.linalg.norm(difference) < 1:
                    self.target_reached[i] = True
                    v = np.zeros(2)
                self.steer_vel[i] = v
            # if all robot reached position, print message
            if all(self.target_reached):
                print("Target position reached!")
                self.target_received = False
                self.target_reached = [False for i in range(self.num_of_robots)]
    
    def avoid_obstacle(self):
        for i in range(self.num_of_robots):
            heading = np.arctan2(self.velocities[i][1], self.velocities[i][0])
            v = self.obstacle.Obstacle_vel(self.target_position, heading, self.positions[i])
            # vx = max(self.min_linear_velx, min(v[0], self.max_linear_velx))
            # vy = max(self.min_linear_vely, min(v[1], self.max_linear_vely))
            # v = np.array([vx, vy])
            self.obs_vel[i] = v

    def weighted_velocities(self, event):
        # compute the weighted velocities and publish them
        self.align()
        self.cohesion()
        self.separation()
        self.steer()
        self.avoid_obstacle()
        # velocity_data = {}

        for i in range(self.num_of_robots):
            weighted_vel = (self.align_vel_weights*self.align_vel[i]
            + self.cohesion_vel_weights*self.cohesion_vel[i]
            + self.separation_vel_weights*self.separation_vel[i]
            + self.steer_vel_weight*self.steer_vel[i]
            + self.obs_vel_weight*self.obs_vel[i])

            v = Twist()
            v.linear.x = max(self.min_linear_velx, min(weighted_vel[0], self.max_linear_velx))
            v.linear.y = max(self.min_linear_vely, min(weighted_vel[1], self.max_linear_vely))
            self.cmd_vel_pub[i].publish(v)
        
        # velocity_data = {'cohesion_vel': self.cohesion_vel_weights*np.linalg.norm(self.cohesion_vel[5]),
        #                 'align_vel': self.align_vel_weights*np.linalg.norm(self.align_vel[5]),
        #                 'separation_vel': self.separation_vel_weights*np.linalg.norm(self.separation_vel[5]),
        #                 'steer_vel': self.steer_vel_weight*np.linalg.norm(self.steer_vel[5]),
        #                 'obs_vel': self.obs_vel_weight*np.linalg.norm(self.obs_vel[5]),
        #                 'time': time.time()}
        
        # self.save_velocity(velocity_data)

    def save_velocity(self, velocity_data):
        with open(self.file_path, 'a') as jsonfile:
            json.dump(velocity_data, jsonfile)
            jsonfile.write('\n')  # Add a newline for better readability


if __name__ == '__main__':
    rospy.init_node('alignment')
    node = Alignment()
    rospy.spin()
