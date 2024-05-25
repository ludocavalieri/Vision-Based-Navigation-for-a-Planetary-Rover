#!/usr/bin/env python3 

import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from sensor_msgs.msg import Image, PointCloud, PointField
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import cv2 as cv
from cv_bridge import CvBridge
bridge = CvBridge()

# Create dense point cloud publisher node: 
class PointCloudNode(): 
    def __init__(self):
        # Initialize node:
        rospy.init_node('point_cloud_node', anonymous = True)
        rospy.loginfo('Point cloud publisher node has started.')

        # Initialize variables: 
        self.left = None
        self.right = None
        self.disparity = None

        # Camera parameters: 
        self.f = 265.5750732421875
        self.u0 = 312.88494873046875
        self.v0 = 196.8916473388672
        self.B = 120.0
        self.Q = np.float32([[1, 0, 0, -self.u0],
                        [0, 1, 0, -self.v0],
                        [0, 0, 0, self.f],
                        [0, 0, -1/self.B, 0]])

        # Initialize stereo matcher for disparity computation:
        self.stereo = cv.StereoBM.create(numDisparities=16, blockSize=15)

        # Generate publisher
        self.PointCloudPub = rospy.Publisher('dense_point_cloud', PointCloud, queue_size = 1)

    # Create listener callbacks: 
    def LeftCallback(self,msg):
        rospy.loginfo('Message received from camera left.')
        self.left = bridge.imgmsg_to_cv2(msg, "rgb8")

    def RightCallback(self,msg):
        rospy.loginfo('Message received from camera right.')
        self.right = bridge.imgmsg_to_cv2(msg, "rgb8")

    # 3D reconstruction: 
    def Reconstruction3D(self):
        rospy.loginfo('Reconstructing 3D space...')
        if self.left is not None and self.right is not None:
            # Convert images to grayscale:
            grayl = cv.cvtColor(self.left, cv.COLOR_RGB2GRAY)
            grayr = cv.cvtColor(self.right, cv.COLOR_RGB2GRAY)

            # Compute disparity:
            self.disparity = self.stereo.compute(grayl,grayr)

            # Compute depth and x and y coordinates: 
            X = []
            Y = []
            Z = []
            h, w = self.left.shape[:2]

            for u in range(w): 
                for v in range(h): 
                    if(self.disparity[v][u] > 0):
                        # depth:
                        z = self.B*self.f/self.disparity[v][u]

                        # x and y: 
                        x = (u - self.u0) * z / self.f
                        y = (v - self.v0) * z / self.f

                        # Generate 3D points: 
                        X.append(x)
                        Y.append(y)
                        Z.append(z)

            # self.points3D = [X, Y, Z]
            self.points3D = [np.array(Z)/100, -np.array(X)/100, -np.array(Y)/100]

            # Publish dense point cloud: 
            PointCloudMessage = PointCloud()
            PointCloudMessage.header = Header()
            PointCloudMessage.header.stamp = rospy.Time.now()
            PointCloudMessage.header.frame_id = "camera_optical_frame"

            for i in range(len(X)):
                # PointCloudMessage.points.append(Point32(X[i]/100, Y[i]/100, Z[i]/100))
                PointCloudMessage.points.append(Point32(Z[i]/100, -X[i]/100, -Y[i]/100))
            self.PointCloudPub.publish(PointCloudMessage)

    # Visualization: 
    def PlotResults(self):
        # Plot disparity:
        if self.disparity is not None:
            plt.figure()
            plt.imshow(self.disparity,'gray')
            plt.title('Disparity Map')

        if self.points3D is not None: 
            # Plot 3D points: 
            fig = plt.figure()
            ax = plt.axes(projection='3d')
            ax.scatter3D(self.points3D[:][0], self.points3D[:][1], self.points3D[:][2], marker='o', s=5, c='r', alpha=0.5)
            plt.title('Dense Point Cloud')
        
        # Show plots:
        plt.show()

    # Node main code:
    def run(self):    
        # Subscribe to desired topics: 
        rospy.Subscriber("/zed2/left/image_rect_color", Image, self.LeftCallback, queue_size=1)
        rospy.Subscriber("/zed2/right/image_rect_color", Image, self.RightCallback, queue_size=1)
        rospy.loginfo("Subscribed to /image_left and /image_right.")

        # Main loop:
        rate = rospy.Rate(1)  # 10 Hz
        while not rospy.is_shutdown():
            self.Reconstruction3D()
            rate.sleep()

        # Show results when node is shutting down:
        # self.PlotResults()

# Run point cloud publisher node: 
if __name__ == '__main__':
    try:
        node = PointCloudNode()
        node.run()
    except rospy.ROSInterruptException:
        pass