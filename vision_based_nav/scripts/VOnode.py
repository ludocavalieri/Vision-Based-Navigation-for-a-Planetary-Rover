#!/usr/bin/env python3 

import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from sensor_msgs.msg import Image, PointCloud
import std_msgs
from geometry_msgs.msg import Point32
import cv2 as cv
from cv_bridge import CvBridge
bridge = CvBridge()

# Create visual odometry node: 
class VisualOdometryNode(): 
    def __init__(self):
        # Initialize node:
        rospy.init_node('dead_reckoning_node', anonymous = True)
        rospy.loginfo('Visual odometry node has started.')

        # General variables:
        self.left1 = None
        self.left2 = None
        self.right1 = None
        self.right2 = None
        self.grayl1 = None
        self.grayl2 = None

        # Create descriptor and matcher:
        self.SIFT = cv.SIFT_create()
        self.BF = cv.BFMatcher(cv.NORM_L2)

        # Create ICP: 
        self.ICP = cv.ppf_match_3d_ICP(100)

        # Initialize stereo matcher for disparity computation:
        self.stereo = cv.StereoBM.create(numDisparities=16, blockSize=15)

        # Descriptor and matcher variables:
        self.kpl1 = None
        self.kpr1 = None
        self.desl1 = None
        self.desr1 = None
        self.matches = None
        self.kpl2 = None
        self.kpr2 = None
        self.desl2 = None
        self.desr2 = None
        self.matches2 = None
        self.matches12 = None
        self.pts2 = None

        # Point cloud variables: 
        self.points3D = None
        self.points3D2 = None

        # Camera parameters: 
        self.K = np.array([[265.5750732421875, 0.0, 312.88494873046875],
                           [0.0, 265.5750732421875, 196.8916473388672],
                           [ 0.0, 0.0, 1.0]])
        self.f = 265.5750732421875
        self.u0 = 312.88494873046875
        self.v0 = 196.8916473388672
        self.B = 120.0
    
    # Create listener callbacks: 
    def LeftCallback(self,msg):
        rospy.loginfo('Message received from camera left.')
        self.left2 = bridge.imgmsg_to_cv2(msg, "rgb8")

    def RightCallback(self,msg):
        rospy.loginfo('Message received from camera right.')
        self.right2 = bridge.imgmsg_to_cv2(msg, "rgb8")

    # Corner detection: 
    def CornerDetector(self): 
        if self.left1 is None or self.left2 is None or self.right1 is None or self.right2 is None:
            rospy.loginfo('Waiting for images...')
            return
        else: 
            # Convert images to grayscale:
            self.grayl1 = cv.cvtColor(self.left1, cv.COLOR_RGB2GRAY)
            grayl2 = cv.cvtColor(self.left2, cv.COLOR_RGB2GRAY)
            self.grayr1 = cv.cvtColor(self.right1, cv.COLOR_RGB2GRAY)
            grayr2 = cv.cvtColor(self.right2, cv.COLOR_RGB2GRAY)

            # Detect corners:
            cornerl1 = cv.cornerHarris(np.float32(self.grayl1), 2, 3, 0.04)
            cornerl2 = cv.cornerHarris(np.float32(grayl2), 2, 3, 0.04)
            cornerr1 = cv.cornerHarris(np.float32(self.grayr1), 2, 3, 0.04)
            cornerr2 = cv.cornerHarris(np.float32(grayr2), 2, 3, 0.04)

            # Apply threshold:
            cornerl1 = np.nonzero(cornerl1 > 0.05 * cornerl1.max())
            cornerl2 = np.nonzero(cornerl2 > 0.05 * cornerl2.max())
            cornerr1 = np.nonzero(cornerr1 > 0.05 * cornerr1.max())
            cornerr2 = np.nonzero(cornerr2 > 0.05 * cornerr2.max())

            # Convert corners to keypoints
            self.kpl1 = [cv.KeyPoint(float(cornerl1[1][i]), float(cornerl1[0][i]), 3) for i in range(len(cornerl1[0]))]
            self.kpr1 = [cv.KeyPoint(float(cornerr1[1][i]), float(cornerr1[0][i]), 3) for i in range(len(cornerr1[0]))]
            self.kpl2 = [cv.KeyPoint(float(cornerl2[1][i]), float(cornerl2[0][i]), 3) for i in range(len(cornerl2[0]))]
            self.kpr2 = [cv.KeyPoint(float(cornerr2[1][i]), float(cornerr2[0][i]), 3) for i in range(len(cornerr2[0]))]

            # Compute SIFT describe for corners: 
            self.kpl1, self.desl1 = self.SIFT.compute(self.grayl1, self.kpl1)
            self.kpr1, self.desr1 = self.SIFT.compute(self.grayr1, self.kpr1)
            self.kpl2, self.desl2 = self.SIFT.compute(grayl2, self.kpl2)
            self.kpr2, self.desr2 = self.SIFT.compute(grayr2, self.kpr2)

    # Matching:
    def MatchingFunction(self): 
        if self.desl1 is None and self.desr1 is None and self.desl2 is None and self.desr2 is None:
            return
        else: 
            rospy.loginfo('Matching corners...')

            # Matching images exploiting SIFT descriptor: 
            self.matches = self.BF.knnMatch(self.desl1,self.desr1, k=2)
            self.matches2 = self.BF.knnMatch(self.desl2,self.desr2, k=2)
            self.matches12 = self.BF.knnMatch(self.desl1,self.desl2, k=2)

            # Apply ratio test as per Lowe's paper:
            good_matches = []
            good_matches2 = []
            good_matches12 = []

            for m, n in self.matches:
                if m.distance < 0.80 * n.distance:
                    good_matches.append(m)
            self.matches = good_matches

            for m, n in self.matches2:
                if m.distance < 0.80 * n.distance:
                    good_matches2.append(m)
            self.matches2 = good_matches2

            for m, n in self.matches12:
                if m.distance < 0.80 * n.distance:
                    good_matches12.append(m)
            self.matches12 = good_matches12

    # Triangulation: 
    def TriangulationFunction(self):
        if self.matches is None or len(self.matches) < 8:
            rospy.loginfo('Not enough matches to perform triangulation.')
            return
        else: 
            rospy.loginfo('Triangulating features...')
            
            # Estimate essential matrix: 
            ptsl1 = np.float32([self.kpl1[m.queryIdx].pt for m in self.matches]).reshape(-1, 1, 2)
            ptsr1 = np.float32([self.kpr1[m.trainIdx].pt for m in self.matches]).reshape(-1, 1, 2)
            ptsl2 = np.float32([self.kpl2[m.queryIdx].pt for m in self.matches2]).reshape(-1, 1, 2)
            ptsr2 = np.float32([self.kpr2[m.trainIdx].pt for m in self.matches2]).reshape(-1, 1, 2)
            E1, mask = cv.findEssentialMat(ptsl1, ptsr1, self.K, method=cv.RANSAC, prob=0.999, threshold=1.0)
            E2, mask2 = cv.findEssentialMat(ptsl2, ptsr2, self.K, method=cv.RANSAC, prob=0.999, threshold=1.0)

            # Estimate camera pose:
            _, R1, t1, _ = cv.recoverPose(E1, ptsl1, ptsr1, self.K, mask=mask)
            _, R2, t2, _ = cv.recoverPose(E2, ptsl2, ptsr2, self.K, mask=mask2)

            # Build the projection matrices for the two cameras:
            Pl1 = np.hstack((np.eye(3), np.zeros((3, 1))))
            Pr1 = np.hstack((R1, t1))
            Pl2 = np.hstack((np.eye(3), np.zeros((3, 1))))
            Pr2 = np.hstack((R2, t2))

            # Convert the projection matrices to the camera coordinate system:
            Pl1 = self.K @ Pl1
            Pr1 = self.K @ Pr1
            Pl2 = self.K @ Pl2
            Pr2 = self.K @ Pr2

            # Triangulate the 3D points:
            points4D = cv.triangulatePoints(Pl1, Pr1, ptsl1, ptsr1)
            points4D2 = cv.triangulatePoints(Pl2, Pr2, ptsl2, ptsr2)
            self.points3D = np.array(points4D[:3] / points4D[3])  # Convert from homogeneous to Cartesian coordinates
            self.points3D2 = np.array(points4D2[:3] / points4D2[3])

    # Motion Estimation: 
    def MotionEstimation3Dto2D(self): 
        if self.points3D is None or self.points3D2 is None:
            return
        else: 
            rospy.loginfo('Estimating motion...')  

            # Disparity and depth: 
            disparity = self.stereo.compute(self.grayl1,self.grayr1).astype(np.float32)
            disparity[disparity == 0.0] = 0.1
            disparity[disparity == -1.0] = 0.1
            depth = self.B*self.f/disparity

            # Extract points matched between left 1 and left 2:
            ptsl_1 = np.float32([self.kpl1[m.queryIdx].pt for m in self.matches12])
            ptsl_2 = np.float32([self.kpl2[m.trainIdx].pt for m in self.matches12])

            # Extract 3D points needed for motion estimation:
            points3D = np.zeros((0,3))
            for indices, (u,v) in enumerate(ptsl_1): 

                # depth:
                z = depth[int(v), int(u)]

                # x and y: 
                x = (u - self.u0) * z / self.f
                y = (v - self.v0) * z / self.f

                # Stacking 3D points: 
                points3D = np.vstack([points3D, np.array([x, y, z])])

            # Solve PnP: 
            _, rvec, t, _ = cv.solvePnPRansac(points3D, ptsl_2, self.K, None)
            R = cv.Rodrigues(rvec)[0]
            print(R)
            print(t)

    # Visualization:
    def PlotResults(self): 
        rospy.loginfo('Displaying results...')

        if self.left1 is not None and self.left2 is not None and self.right1 is not None and self.right2 is not None:
            # Highlight corners in the images and draw matches:
            plt.figure()
            img = cv.drawMatches(self.left1,self.kpl1,self.right1,self.kpr1,self.matches,None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            plt.imshow(img)
            plt.title('Matched Features - t0')
            plt.figure()
            img2 = cv.drawMatches(self.left2,self.kpl2,self.right2,self.kpr2,self.matches2,None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            plt.imshow(img2)
            plt.title('Matched Features - t0+dt')
            plt.show()
        else:
            rospy.loginfo('No image to display.')

    # Node main code:
    def run(self):    
        # Subscribe to desired topics: 
        rospy.Subscriber("/zed2/left/image_rect_color", Image, self.LeftCallback, queue_size=1)
        rospy.Subscriber("/zed2/right/image_rect_color", Image, self.RightCallback, queue_size=1)
        rospy.loginfo("Subscribed to /image_left and /image_right.")

        # Main loop:
        rate = rospy.Rate(1)  # 10 Hz
        while not rospy.is_shutdown():
            self.CornerDetector()
            self.MatchingFunction()
            self.TriangulationFunction()
            self.MotionEstimation3Dto2D()
            if self.left2 is not None and self.right2 is not None:
                self.left1 = self.left2
                self.right1 = self.right2
            rate.sleep()

        # Plot results: 
        self.PlotResults()

# Run dead reckoning node: 
if __name__ == '__main__':
    try:
        node = VisualOdometryNode()
        node.run()
    except rospy.ROSInterruptException:
        pass