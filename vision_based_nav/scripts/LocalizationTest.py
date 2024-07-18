#!/usr/bin/env python3 

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import message_filters

# Define dead_reckoning node:
class LocalizationTest():
    def __init__(self):
        # Initialize node:
        rospy.init_node('testing_node', anonymous = True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize state variables:
        self.ground_truth_x = [0] 
        self.ground_truth_y = [0]
        self.ground_truth_theta = [0]
        self.odometry_x = [0]
        self.odometry_y = [0]
        self.odometry_theta = [0]

        # Initialize time vectors
        self.timeVO = []
        self.timeGT = []
        self.timeZEDVO = []

        # Initialize error vectors: 
        self.error = []
        self.errortheta = []
        self.errordtheta = []

    def OdomCallback(self,msgGT,msgVO):
        rospy.loginfo("Messages received from ground truth and custom odometry topics.")
        # Update time vector: 
        t = rospy.Time.now().to_sec()
        self.timeGT.append(t)

        # GT output
        self.ground_truth_x.append(msgGT.pose.pose.position.x)
        self.ground_truth_y.append(msgGT.pose.pose.position.y)
        self.ground_truth_theta.append(np.arctan2(self.ground_truth_y[-1]-self.ground_truth_y[-2], self.ground_truth_x[-1]-self.ground_truth_x[-2]))

        # Update time vector: 
        self.timeVO.append(t)

        # Transform vector: 
        transform = self.tf_buffer.lookup_transform("base_link", msgVO.header.frame_id, msgVO.header.stamp, rospy.Duration(1.0))
        msgVO = tf2_geometry_msgs.do_transform_pose(msgVO, transform)

        # VO output
        self.odometry_x.append(msgVO.pose.position.x)
        self.odometry_y.append(msgVO.pose.position.y)
        self.odometry_theta.append(np.arctan2(self.odometry_y[-1]-self.odometry_y[-2], self.odometry_x[-1]-self.odometry_x[-2]))

    def ComputeError(self):
        # Compute errors:
        for i in range(len(self.timeGT)): 
            er = np.sqrt((self.ground_truth_x[i]-self.odometry_x[i])**2+(self.ground_truth_y[i]-self.odometry_y[i])**2)
            ertheta = abs(self.ground_truth_theta[i]-self.odometry_theta[i])
            self.error.append(er)  
            self.errortheta.append(ertheta)

        # Final error: 
        print("Final error: ", self.error[-1])

        # Length of traverse:
        traverse = 0
        for i in range(len(self.timeGT)-1):
            traverse = traverse + np.sqrt((self.ground_truth_x[i+1]-self.ground_truth_x[i])**2+(self.ground_truth_y[i+1]-self.ground_truth_y[i])**2)
        print(f'Length of traverse: {traverse} m')

        # RMSE:
        N = len(self.timeGT)
        RMSE = np.sqrt(np.mean(np.array(self.error)**2))
        print(f'RMSE: {RMSE} m')  

        # RMSE relative to path length: 
        RMSE_percent = RMSE/traverse*100
        print(f'Percentage RMSE: {RMSE_percent} %')

        # Mean orientation error: 
        for i in range(len(self.errortheta)): 
            if self.errortheta[i] > np.pi/2: 
                self.errortheta[i] = 0
        meanthetaer = (sum(self.errortheta)/len(self.errortheta))*(180/np.pi)/traverse
        print(f'Mean orientation error over traverse length: {meanthetaer} deg/m')

    def PlotResults(self): 
        # Trajectory comparison:
        fig1 = plt.figure()    
        plt.plot(self.ground_truth_x, self.ground_truth_y, label='Ground Truth',color='red') # Plot the ground truth trajectory  
        plt.plot(self.odometry_x, self.odometry_y, label='Visual Odometry',color='blue') # Plot the trajectory reconstructed with visual odometry - custom
        plt.xlabel('X position [m]')
        plt.ylabel('Y position [m]')
        plt.title('Trajectory Comparison')
        plt.legend()
        plt.grid(True)

        # Error plot:
        fig2, (ax1, ax2) = plt.subplots(1, 2)  
        ax1.plot(self.timeGT, self.error, label='Error', color='red',) 
        ax1.set_xlabel('Time [sec]')
        ax1.set_ylabel('Error [m]')
        ax1.set_title('Trajectory Error')
        ax1.legend()
        ax1.grid(True)
        ax2.plot(self.timeGT, np.array(self.errortheta)*180/np.pi, label='Orientation Error',color='red') 
        ax2.set_xlabel('Time [sec]')
        ax2.set_ylabel('Error [deg]')
        ax2.set_title('Orientation Error')
        ax2.legend()
        ax2.grid(True)

        plt.show()

    def run(self): 

        # Create subscribers:
        GT_sub = message_filters.Subscriber('/ground_truth', Odometry)
        VO_sub = message_filters.Subscriber('/custom_odom', PoseStamped)

        # Synchronize messages:
        ts = message_filters.ApproximateTimeSynchronizer([GT_sub, VO_sub], 10, 0.1)
        ts.registerCallback(lambda msgGT, msgVO: self.OdomCallback(msgGT, msgVO))

        rate = rospy.Rate(0.5) # [Hz]
        while not rospy.is_shutdown():            
            rate.sleep()
    
        self.ComputeError()
        self.PlotResults()

# Run dead reckoning node: 
if __name__ == '__main__':
    try:
        node = LocalizationTest()
        node.run()
    except rospy.ROSInterruptException:
        # Plot results: 
        node.ComputeError()
        node.PlotResults()
        pass