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
        rospy.init_node('dead_reckoning_node', anonymous = True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize state variables:
        self.ground_truth_x = [0] 
        self.ground_truth_y = [0]
        self.odometry_x = [0]
        self.odometry_y = [0]
        self.odometry_x_int = None
        self.odometry_y_int = None

        # Initialize time vectors
        t0 = rospy.Time.now().to_sec()
        self.timeVO = [t0]
        self.timeGT = [t0]
        self.timeZEDVO = [t0]

        # Initialize error vectors: 
        self.error = []
        self.errorx = []
        self.errory = []

    def OdomCallback(self,msgGT,msgVO):
        # Update time vector: 
        t = rospy.Time.now().to_sec()
        self.timeGT.append(t)

        # GT output
        self.ground_truth_x.append(msgGT.pose.pose.position.x)
        self.ground_truth_y.append(msgGT.pose.pose.position.y)

        # Update time vector: 
        self.timeVO.append(t)

        # Transform vector: 
        transform = self.tf_buffer.lookup_transform("base_link", msgVO.header.frame_id, msgVO.header.stamp, rospy.Duration(1.0))
        msgVO = tf2_geometry_msgs.do_transform_pose(msgVO, transform)

        # VO output
        self.odometry_x.append(msgVO.pose.position.x)
        self.odometry_y.append(msgVO.pose.position.y)

    def ComputeError(self):
        # Interpolate odometry data: 
        self.odometry_x_int = np.interp(np.array(self.timeGT),np.array(self.timeVO),np.array(self.odometry_x))
        self.odometry_y_int = np.interp(np.array(self.timeGT),np.array(self.timeVO),np.array(self.odometry_y))

        # Compute errors:
        for i in range(len(self.odometry_x_int)): 
            er = np.sqrt((self.ground_truth_x[i]-self.odometry_x_int[i])**2+(self.ground_truth_y[i]-self.odometry_y_int[i])**2)
            erx = abs(self.ground_truth_x[i]-self.odometry_x_int[i])
            ery = abs(self.ground_truth_y[i]-self.odometry_y_int[i])
            self.error.append(er)  
            self.errorx.append(erx)
            self.errory.append(ery)

        # Final error: 
        print("Final error: ", self.error[-1])

        # Length of traverse:
        traverse = 0
        for i in range(len(self.timeGT)-1):
            traverse = traverse + np.sqrt((self.ground_truth_x[i+1]-self.ground_truth_x[i])**2+(self.ground_truth_y[i+1]-self.ground_truth_y[i])**2)
        print("Length of traverse: ", traverse)

        # Root Mean Square Error:
        N = len(self.timeGT)
        RMSE = 0
        for i in range(N-1):
            RMSE = RMSE + self.error[i]**2
        RMSE = np.sqrt(RMSE/N)
        print("RMSE: ", RMSE)  

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
        ax1.plot(self.timeGT, self.error, label='Error',color='red') 
        ax1.set_xlabel('Time [sec]')
        ax1.set_ylabel('Error [m]')
        ax1.set_title('Trajectory Error')
        ax1.legend()
        ax1.grid(True)
        ax2.plot(self.timeGT, self.errorx, label='Error on x',color='blue') 
        ax2.plot(self.timeGT, self.errory, label='Error on y',color='green')
        ax2.set_xlabel('Time [sec]')
        ax2.set_ylabel('Error [m]')
        ax2.set_title('Errors on Components')
        ax2.legend()
        ax2.grid(True)

        plt.show()

    def run(self): 
        rate = rospy.Rate(1) # [Hz]
        while not rospy.is_shutdown(): 
            # Create subscribers:
            GT_sub = message_filters.Subscriber('/ground_truth', Odometry)
            VO_sub = message_filters.Subscriber('/custom_odom', PoseStamped)

            # Synchronize messages:
            ts = message_filters.ApproximateTimeSynchronizer([GT_sub, VO_sub], 10, 0.1)
            ts.registerCallback(lambda msgGT, msgVO: self.OdomCallback(msgGT, msgVO))
            
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