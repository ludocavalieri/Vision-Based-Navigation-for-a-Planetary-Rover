#!/usr/bin/env python3 

import rospy
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import Image, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
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
        self.depth = None
        self.X = []
        self.Y = []
        self.Z = []

        # Camera parameters: 
        self.f = 527.2972398956961
        self.u0 = 658.8206787109375
        self.v0 = 372.25787353515625
        self.B = 120.0
        self.Q = np.float32([[1, 0, 0, -self.u0],
                        [0, 1, 0, -self.v0],
                        [0, 0, 0, self.f],
                        [0, 0, -1/self.B, 0]])
        
        # Costmap parameters:
        self.resolution = 0.1  # Costmap resolution in meters
        self.width = 400  # Number of cells in the width
        self.height = 400  # Number of cells in the height

        # Initialize stereo matcher for disparity computation:
        self.stereo = cv.StereoBM.create(numDisparities=5*16, blockSize=13)

        # Generate publisher
        self.PointCloudPub = rospy.Publisher('dense_point_cloud', PointCloud, queue_size = 1)
        self.CostmapPub = rospy.Publisher('costmap_custom', OccupancyGrid, queue_size=1)

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
            self.disparity = self.stereo.compute(grayl,grayr)/16
            self.disparity[self.disparity <= 0.0] = 0.1
            self.depth = self.B*self.f/self.disparity

            # Compute depth and x and y coordinates: 
            X = []
            Y = []
            Z = []
            h, w = self.left.shape[:2]
            min_depth = 200  # Adjust this value as needed
            max_depth = 250000.0  # Adjust this value as needed 

            for u in range(w): 
                for v in range(h): 
                    # depth:
                    z = self.depth[int(v), int(u)]
                    # Filter out points that do not fall within the specified depth range:
                    if z > min_depth and z < max_depth:
                        # z:
                        z = self.depth[v][u]

                        # x and y: 
                        x = (u - self.u0) * z / self.f
                        y = (v - self.v0) * z / self.f

                        # Generate 3D points: 
                        self.X.append(x) # in mm 
                        self.Y.append(y) # in mm
                        self.Z.append(z) # in mm

            # Publish dense point cloud: 
            PointCloudMessage = PointCloud()
            PointCloudMessage.header = Header()
            PointCloudMessage.header.stamp = rospy.Time.now()
            PointCloudMessage.header.frame_id = "camera_link"

            for i in range(len(self.X)):
                PointCloudMessage.points.append(Point32(self.X[i]/1000, self.Y[i]/1000, self.Z[i]/1000))
            self.PointCloudPub.publish(PointCloudMessage)

    # Generate costmap: 
    def PointsTocostmap(self):
        rospy.loginfo('Generating costmap...')

        # Generate costmap message: 
        costmap = OccupancyGrid()
        costmap.header = Header()
        costmap.header.stamp = rospy.Time.now()
        costmap.header.frame_id = "camera_link2"
        costmap.info = MapMetaData()
        costmap.info.resolution = self.resolution
        costmap.info.width = self.width
        costmap.info.height = self.height
        costmap.info.origin.position.x = -self.width * self.resolution / 2.0
        costmap.info.origin.position.y = -self.height * self.resolution / 2.0
        costmap.info.origin.position.z = 0
        costmap.info.origin.orientation.w = 1

        # Initialize the costmap data with -1 (unknown)
        costmap.data = [-1] * (self.width * self.height)

        # Generate costmap: 
        for i in range(len(self.X)): 
            # Extract coordinates: 
            x = self.X[i]/1000
            y = -self.Z[i]/1000

            # Define grid coordinates: 
            grid_x = int((x - costmap.info.origin.position.x) / (self.resolution))
            grid_y = int((y - costmap.info.origin.position.y) / (self.resolution/np.cos(0.3))) # might be times the cosine of the angle between base_link and camera_link

            # Assign occupancy probabilities:
            if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
                index = grid_y * self.width + grid_x 
                costmap.data[index] = 100

        # Publish costmap:
        self.CostmapPub.publish(costmap)

    # Visualization: 
    def PlotResults(self):
        # Plot disparity:
        if self.disparity is not None:
            plt.figure()
            plt.imshow(self.disparity,'jet')
            plt.colorbar()
            plt.title('Disparity Map')

        if self.depth is not None:
            plt.figure()
            plt.imshow(self.depth,'jet')
            plt.colorbar()
            plt.title('Depth Map')
        
        # Show plots:
        plt.show()

    # Node main code:
    def run(self):    
        # Subscribe to desired topics: 
        rospy.Subscriber("/zed2/left/image_rect_color", Image, self.LeftCallback, queue_size=1)
        rospy.Subscriber("/zed2/right/image_rect_color", Image, self.RightCallback, queue_size=1)
        rospy.loginfo("Subscribed to /image_left and /image_right.")

        # Main loop:
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.Reconstruction3D()
            self.PointsTocostmap()
            rate.sleep()

# Run point cloud publisher node: 
if __name__ == '__main__':
    try:
        node = PointCloudNode()
        node.run()
    except rospy.ROSInterruptException:
        # Show results when node is shutting down:
        # node.PlotResults()
        pass