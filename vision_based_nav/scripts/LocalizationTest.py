#!/usr/bin/env python3 

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import os
from tf.transformations import euler_from_quaternion

# Define the path to the folder on your desktop where you want to save the plots
desktop_folder = os.path.expanduser("~/Desktop/Tech Team/Localization Test")

# Define dead_reckoning node:
class LocalizationTest():
    def __init__(self):
        # Initialize node:
        rospy.init_node('dead_reckoning_node', anonymous = True)
    
        # Initialize state variables:
        self.ground_truth_x = [0]
        self.ground_truth_y = [0]
        self.ground_truth_theta = [0]
        self.odometry_x = [0]
        self.odometry_y = [0]
        self.odometry_theta = [0]

        # Initialize time vectors
        t0 = rospy.Time.now().to_sec()
        self.timeVO = [t0]
        self.timeGT = [t0]

    def GT_callback(self,msg): 
        # Update time vector: 
        t = rospy.Time.now().to_sec()
        self.timeGT.append(t)

        # GT output
        self.ground_truth_x.append(msg.pose.pose.position.x)
        self.ground_truth_y.append(msg.pose.pose.position.y)
        angles = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.ground_truth_theta.append(angles[2])

    def plot_results(self): 
        # Trajectory comparison:
        fig1 = plt.figure(figsize = (8, 6))    
        plt.plot(self.ground_truth_x, self.ground_truth_y, label='Ground Truth',color='blue') # Plot the ground truth trajectory  
        # plt.plot(self.odometry_x, self.odometry_y, label='Wheel Odometry',color='red') # Plot the trajectory reconstructed with wheel odometry
        plt.xlabel('X position (m)')
        plt.ylabel('Y position (m)')
        plt.title('Trajectory Comparison')
        plt.legend()
        plt.grid(True)

        # Orientation comparison: 
        fig2 = plt.figure(figsize = (8, 6))
        plt.plot(self.timeGT,self.ground_truth_theta, label='Ground Truth',color='blue') # Plot the ground truth orientation
        # plt.plot(self.timeVO,self.odometry_theta, label='Wheel Odometry',color='red') # Plot the orientation reconstructed with wheel odometry
        plt.ylabel('theta [rad]')
        plt.title('Orientation Comparison')
        plt.legend()
        plt.grid(True)

        plt.show()
        plt.pause(0.1)  # Add a short delay

    def run(self): 
        rate = rospy.Rate(20) # [Hz]
        while not rospy.is_shutdown(): 
            GT_sub = rospy.Subscriber('/ground_truth', Odometry, lambda x: self.GT_callback(x),queue_size=1)
            # VO_sub = rospy.Subscriber('/odom', Odometry, lambda x: self.WO_callback(x),queue_size=1) # Might change with /odom
            rate.sleep()
        
        # Plot results: 
        node.plot_results()

# Run dead reckoning node: 
if __name__ == '__main__':
    try:
        node = LocalizationTest()
        node.run()
    except rospy.ROSInterruptException:
        pass