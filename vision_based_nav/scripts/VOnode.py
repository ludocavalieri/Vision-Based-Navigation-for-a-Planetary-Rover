#!/usr/bin/env python3 

import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf
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

        # Select detector: 
        self.detector = 'orb' # options: Harris, SIFT, SURF, ORB

        # Create descriptor and matcher:
        if self.detector == 'sift' or self.detector == 'harris': 
            self.det = cv.SIFT_create()
            self.BF = cv.BFMatcher(cv.NORM_L2)
        elif self.detector == 'surf': 
            self.det = cv.xfeatures2d.SURF_create()
            self.BF = cv.BFMatcher(cv.NORM_L2)
        elif self.detector == 'orb': 
            self.det = cv.ORB_create()
            self.BF = cv.BFMatcher(cv.NORM_HAMMING2)
        else: 
            print('Unrecognized detector.')

        # Select stereo matcher: 
        self.stereomatcher = 'BM'

        # Initialize stereo matcher for disparity computation:
        if self.stereomatcher == 'BM': 
            self.stereo = cv.StereoBM.create(numDisparities=5*16, blockSize=13) # TODO: tune parameters
        elif self.stereomatcher == 'SGBM': 
            self.stereo = cv.StereoSGBM.create(numDisparities=5*16, blockSize=13, P1=8*3*13**2, P2= 32*3*13**2, mode=cv.STEREO_SGBM_MODE_SGBM_3WAY) # TODO: tune parameters

        # Descriptor variables:
        self.kpl1 = None 
        self.kpr1 = None 
        self.desl1 = None
        self.desr1 = None
        self.kpl2 = None 
        self.desl2 = None

        # Matcher variables:
        self.matches = None
        self.matches12 = None
        self.pts2 = None

        # Depth variables: 
        self.points3D = None
        self.disparity = None
        self.depth = None

        # Camera parameters: 
        self.f = 527.2972398956961
        self.u0 = 658.8206787109375
        self.v0 = 372.25787353515625
        self.B = 120.0
        self.K = np.array([[self.f, 0.0, self.u0],
                           [0.0, self.f, self.v0],
                           [ 0.0, 0.0, 1.0]])

        # Motion estimation variables: 
        self.R = None
        self.t = None
        self.Rtot = np.eye(3)
        self.x0 = np.array([[0, 0, 0]]).reshape(3,1)
        self.trajectory = np.array([[0, 0, 0]])

        # Odometry publisher: 
        self.PosePub = rospy.Publisher("/custom_odom", PoseStamped, queue_size=1)
    
    # Create listener callbacks: 
    def LeftCallback(self,msg):
        rospy.loginfo('Message received from camera left.')
        self.left2 = bridge.imgmsg_to_cv2(msg, "rgb8")

    def RightCallback(self,msg):
        rospy.loginfo('Message received from camera right.')
        self.right2 = bridge.imgmsg_to_cv2(msg, "rgb8")

    # Feature detection: 
    def FeatureDetector(self): 
        if self.left1 is None or self.left2 is None or self.right1 is None or self.right2 is None:
            rospy.loginfo('Waiting for images...')
            return
        else:
            # Convert images to grayscale:
            self.grayl1 = cv.cvtColor(self.left1, cv.COLOR_RGB2GRAY)
            self.grayl2 = cv.cvtColor(self.left2, cv.COLOR_RGB2GRAY)
            self.grayr1 = cv.cvtColor(self.right1, cv.COLOR_RGB2GRAY)

            # Feature detection algorithm: 
            if self.detector == 'harris':
                # Detect corners:
                cornerl1 = cv.cornerHarris(np.float32(self.grayl1), 2, 3, 0.04)
                cornerl2 = cv.cornerHarris(np.float32(self.grayl2), 2, 3, 0.04)
                cornerr1 = cv.cornerHarris(np.float32(self.grayr1), 2, 3, 0.04)
                
                # Apply threshold:
                cornerl1 = np.nonzero(cornerl1 > 0.05 * cornerl1.max())
                cornerl2 = np.nonzero(cornerl2 > 0.05 * cornerl2.max())
                cornerr1 = np.nonzero(cornerr1 > 0.05 * cornerr1.max())
                
                # Convert corners to keypoints
                self.kpl1 = [cv.KeyPoint(float(cornerl1[1][i]), float(cornerl1[0][i]), 3) for i in range(len(cornerl1[0]))]
                self.kpr1 = [cv.KeyPoint(float(cornerr1[1][i]), float(cornerr1[0][i]), 3) for i in range(len(cornerr1[0]))]
                self.kpl2 = [cv.KeyPoint(float(cornerl2[1][i]), float(cornerl2[0][i]), 3) for i in range(len(cornerl2[0]))]
                
                # Compute SIFT descriptor for corners: 
                self.kpl1, self.desl1 = self.det.compute(self.grayl1, self.kpl1)
                self.kpr1, self.desr1 = self.det.compute(self.grayr1, self.kpr1)
                self.kpl2, self.desl2 = self.det.compute(self.grayl2, self.kpl2)
            elif self.detector == 'sift' or self.detector == 'surf' or self.detector == 'orb':
                # Use SIFT/SURF/ORB to detect and describe features:
                self.kpl1, self.desl1 = self.det.detectAndCompute(self.grayl1, None)
                self.kpl2, self.desl2 = self.det.detectAndCompute(self.grayl2, None)
                self.kpr1, self.desr1 = self.det.detectAndCompute(self.grayr1, None)
            
    # Matching:
    def MatchingFunction(self): 
        if self.desl1 is None and self.desr1 is None and self.desl2 is None:
            return
        else: 
            rospy.loginfo('Matching features...')

            # Matching images exploiting SIFT descriptor: 
            self.matches = self.BF.knnMatch(self.desl1,self.desr1, k=2)
            self.matches12 = self.BF.knnMatch(self.desl1,self.desl2, k=2)
            self.matches = sorted(self.matches, key = lambda x:x[0].distance)
            self.matches12 = sorted(self.matches12, key = lambda x:x[0].distance)
            
            print('Number of matches before filtering - l1/r1:', len(self.matches))
            print('Number of matches before filtering - l1/l2:', len(self.matches12))

            # Apply ratio test as per Lowe's paper:
            good_matches = []
            good_matches12 = []

            for m, n in self.matches:
                if m.distance < 0.75 * n.distance:
                    good_matches.append(m)
            self.matches = good_matches

            for m, n in self.matches12:
                if m.distance < 0.75 * n.distance:
                    good_matches12.append(m)
            self.matches12 = good_matches12

            print('Number of matches after filtering - l1/r1:', len(self.matches))
            print('Number of matches after filtering - l1/l2:', len(self.matches12))

    # Motion Estimation: 
    def MotionEstimation3Dto2D(self): 
        if self.matches12 is None:
            return
        else: 
            rospy.loginfo('Estimating motion...')  

            # Disparity and depth: 
            disparity = self.stereo.compute(self.grayl1,self.grayr1)/16
            disparity[disparity <= 0.0] = 0.1
            self.disparity = disparity
            self.depth = self.B*self.f/disparity

            # Extract points matched between left 1 and left 2:
            ptsl_1 = np.float32([self.kpl1[m.queryIdx].pt for m in self.matches12])
            ptsl_2 = np.float32([self.kpl2[m.trainIdx].pt for m in self.matches12])

            # Extract 3D points needed for motion estimation:
            points3D = np.zeros((0,3))
            min_depth = 200  # Adjust this value as needed
            max_depth = 250000.0  # Adjust this value as needed 
            valid_indices = []

            for indices, (u,v) in enumerate(ptsl_1): 
                # Depth:
                z = self.depth[int(v), int(u)]

                # Filter out points that do not fall within the specified depth range:
                if z > min_depth and z < max_depth:
                    # x and y: 
                    x = (u-self.u0)*z/self.f
                    y = (v-self.v0)*z/self.f

                    # Stacking 3D points: 
                    points3D = np.vstack([points3D, np.array([x, y, z])])

                    # Store valid indices: 
                    valid_indices.append(indices)

            # Define 3D points and 2D points to keep: 
            self.points3D = points3D
            ptsl2_depthlimited = ptsl_2[valid_indices]

            # Solve PnP:
            try: 
                _, rvec, self.t, inliers = cv.solvePnPRansac(self.points3D, ptsl2_depthlimited, self.K, None)
                self.R = cv.Rodrigues(rvec)[0]
                print(self.R, self.t)
                print('Number of inliers: {}/{} matched features'.format(len(inliers), len(self.matches12)))
            except: 
                self.R = np.eye(3)
                self.t = np.zeros((3,1))

    # Trajectory Reconstruction: 
    def TrajectoryReconstruction(self): 
        if self.R is None and self.t is None:
            return
        else: 
            rospy.loginfo('Updating position...')  

            # Apply transform to get new position:
            self.Rtot = self.Rtot @ self.R.T
            self.x0 = self.x0+np.dot(self.Rtot,-self.t)
            self.trajectory = np.vstack([self.trajectory, self.x0.T])

            # Initialize odom message:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "camera_link"
            
            # Convert the position to float:
            pose_msg.pose.position.x = float(self.x0[0])/1000
            pose_msg.pose.position.y = float(self.x0[1])/1000
            pose_msg.pose.position.z = float(self.x0[2])/1000
            
            # Ensure orientation is a valid quaternion:
            pose_msg.pose.orientation.x = 1.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 0.0

            # Publish message:
            self.PosePub.publish(pose_msg)
            print(pose_msg)

            # Broadcast transform: 
            Mat = np.eye(4)
            Mat[:3,:3] = self.Rtot
            quat = tf.transformations.quaternion_from_matrix(Mat)
            tf2Broadcast = tf2_ros.TransformBroadcaster()
            tf2Stamp = TransformStamped()
            tf2Stamp.header.stamp = rospy.Time.now()
            tf2Stamp.header.frame_id = 'odom'
            tf2Stamp.child_frame_id = 'base_footprint'
            tf2Stamp.transform.translation.x = self.x0[0]/1000
            tf2Stamp.transform.translation.y = self.x0[1]/1000
            tf2Stamp.transform.translation.z = self.x0[2]/1000
            tf2Stamp.transform.rotation.x = quat[0]
            tf2Stamp.transform.rotation.y = quat[1]
            tf2Stamp.transform.rotation.z = quat[2]
            tf2Stamp.transform.rotation.w = quat[3]
            tf2Broadcast.sendTransform(tf2Stamp)

    # Visualization:
    def PlotResults(self): 
        rospy.loginfo('Displaying results...')

        # Feature detection and matching results:
        if self.left1 is not None and self.left2 is not None and self.right1 is not None:
            plt.figure()
            img = cv.drawMatches(self.left1,self.kpl1,self.right1,self.kpr1,self.matches,None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            plt.imshow(img)
            plt.title('Matched Features - left t0 and right t0')
            plt.figure()
            img = cv.drawMatches(self.left1,self.kpl1,self.left2,self.kpl2,self.matches12,None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            plt.imshow(img)
            plt.title('Matched Features - left t0 and left t0+dt')

        # Plot disparity:
        if self.disparity is not None:
            plt.figure()
            plt.imshow(self.disparity,'jet')
            plt.colorbar()
            plt.title('Disparity Map')

        # Plot depth:
        if self.depth is not None:
            plt.figure()
            plt.imshow(self.depth,'jet')
            plt.colorbar()
            plt.title('Depth Map')

        # Plot Trajectory: 
        if self.trajectory is not None:
            fig = plt.figure()
            ax = plt.axes(projection='3d')
            xs = self.trajectory[:, 0]
            ys = self.trajectory[:, 1]
            zs = self.trajectory[:, 2]
            plt.plot(xs, ys, zs, c='blue')
            plt.title('Trajectory')
            
        # Shows results:
        plt.show()

    # Node main code:
    def run(self):    
        # Subscribe to desired topics: 
        rospy.Subscriber("/zed2/left/image_rect_color", Image, self.LeftCallback, queue_size=1)
        rospy.Subscriber("/zed2/right/image_rect_color", Image, self.RightCallback, queue_size=1)
        rospy.loginfo("Subscribed to /image_left and /image_right.")

        # Main loop:
        rate = rospy.Rate(0.5)  # 0.5 Hz -> if the frequency is too high, the time step is too small and the disparity computation produces larger errors, if it's too low the trajectory isn't sufficiently smooth (a lot depends on simulated environment, few features -> lower frequency is needed)
        while not rospy.is_shutdown():
            # Visual odometry procedure:
            self.FeatureDetector()
            self.MatchingFunction()
            self.MotionEstimation3Dto2D()
            self.TrajectoryReconstruction()

            # Update image at t0: 
            self.left1 = self.left2
            self.right1 = self.right2

            # Repeat:
            rate.sleep()
        
        self.PlotResults() # bag

# Run dead reckoning node: 
if __name__ == '__main__':
    try:
        node = VisualOdometryNode()
        node.run()
    except rospy.ROSInterruptException:
        # Plot results: 
        node.PlotResults() # simulation
        pass