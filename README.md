# Vision-based Navigation for a Planetary Rover

This repository contains the Google Colab notebook and ROS package used to perform visual odometry with the ESA Katwijk beach dataset and in a simulated environment, respectively.

# Abstract

Communication and computing performances of planetary rovers limit their autonomous traverse distances. Moreover, for missions to Mars, manual driving is unfeasible: not only does the round trip communication time between Earth and Mars takes from 5 to 20 minutes, but contact with ground can be reached only twice per Sol. Even with the help of orbital data, planning long traverses, and avoiding all local obstacles the rover may encounter on its path, proves to be nearly an impossible task. Such rovers are expensive and out of reach for physical help, and therefore, any collision critically endangers the mission's success. Neverthless, considering future missions' programs, there is the necessity for a high degree of autonomy of these robotic assets.

This work aims to implement and test a complete Stereo Visual Odometry (SVO) pipeline with 3D-to-2D motion estimation for the navigation of a planetary rover. 
Different feature detectors and descriptors will be used, namely: 
- SIFT;
- FAST + SIFT;
- ORB;
- BRISK.

Different stereo matching techniques will be compared, specifically: 
- Block Matching;
- Semi-global Block Matching.

Beside evaluating the difference configurations performance, the focus of this work is to compare and analyze the proposed navigation strategy's performance in both a Real and Synthetic environment. The *Real* environment is represented by the "Katwijk Beach Planetary Rover Dataset', provided by ESA and available at [Katwijk Beach Planetary Rover Dataset](https://robotics.estec.esa.int/datasets/katwijk-beach-11-2015/). The *Synthetic* environment is a custom-made Gazebo world, reproducing a Mars-like environment. A second custom-made world, quite similar to the Katwijk beach scenario, is also provided, however it is less accurate than the other. The simulations in the synthetic environment were performed through the Robot Operating System (ROS).

This research was inspired by the fact that, given the lack of stereo-VO datasets acquired in a planetary analog environment, maby studies pertaining to this research field rely solely on simulation environments. However, to the authors' knowledge, there are no studies that directly address the problem of performance transferability from the synthetic environment to the real-world deployment of the application.

# Evaluation Metrics

To evaluate the VO-pipeline performances and compare them between the 'Real' and 'Synthetic' environments, the [official KITTI leaderboard performance metrics](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) were adopted:

**Geometric distance history (m):**
point by point euclidean distance between the corresponding ground truth and estimated trajectory points

**Angular difference history (deg):**
point by point absolute difference between ground truth orientation and estimated orientation

**Root Mean Square Error (RMSE) relative to path length (%):**
RMSE for the whole trajectory divided by the total path length

**Mean angular difference relative to path length (deg/m):**
Mean absolute orientation error along the whole trajectory divided by the total path length

# Running the code

## Visual odometry with Katwijk dataset

## Dataset 
For testing of the the 'Real Environment' performances the 'Katwijk beach planetary rover dataset' was used. The dataset was collected on the Katwijk beach in Netherlands near the ESA-ESTEC site. The objective was to create a reference dataset for development and testing of future ESA's Mars Exploration Rovers capabilities in an analog terrain. For this reason, several artificial boulders were placed in order to reproduce the measured rock density correspondent to the deputy landing site of ExoMars missions. Morover the Heavy Duty Planetary Rover (HDPR) was used to collect data since it represents a good model of the ExoMars rovers. It is a rocker-bogie-style rover with multiple sensors (LocCam, PanCam, LiDAR, ToF camera). The data used in this work where the 1024x768 RGB images from the PointGrey Bumblebee2 stereocamera and the GPS RTK collected data inUTM-31 frame, to reconstruct the ground truth.

### Perform visual odometry and evaluate its performance 
The pipeline was implemented in Google Colab and its available in the notebook 'KatwijkVO' available in the repository. To run the code it is necessary to have the Transverse 3 - Part 1 dataset downloaded on Google Drive and mount the drive, then setup the correct path to the folder containing the dataset. The first set of cells implements the pipeline functions and briefly comments hw they work. The second parth of the notebook shows of the test of the 8 combinations of the different descriptor and matcher and their respective performances.

![Katwijk rover specifications.](<vision_based_nav/figures/Katwijk rover.png>)

## Visual odometry in simulated environment

### Start the simulation
The simulation environment is specified in [model.sdf](vision_based_nav/models/mars_like_environment/model.sdf), where it is possible to select *katwijk simulated.dae* amd *model.dae*. The former is a basic reproduction of the Katwijk beach portion in which the real dataset is acquired, while the latter is a slightly more complex and realistic reproduction of a planetary terrain. The node [RobotController.py](vision_based_nav/scripts/RobotController.py) enables sending */cmd_vel* messages to the LEO rover model using the arrow keys.
To launch the simulation environment (both Gazebo and Rviz), run the following commands from terminal: 
~~~
roslaunch vision_based_nav gazebo_model.launch 
rosrun vision_based_nav RobotController.py
~~~
>[!NOTE]
>To maintain the controller active, click on the GUI.

![Gazebo world.](<vision_based_nav/figures/Gazebo.png>) 
![RViz visualization.](<vision_based_nav/figures/RViz.png>)

### Perform visual odometry and evaluate its performance 
The Visual Odometry procedure is implemented in [VOnode.py](vision_based_nav/scripts/VOnode.py), which subscribes to the image topics on which the ZED 2i stereo camera mounted on the model publishes the acquired images. It then publishes the retrieved pose as a PoseStamped message and the transform between the world  frame and the rover frame as a tf. 
To test the pipeline's performance, run [LocalizationTest.py](vision_based_nav/scripts/LocalizationTest.py), which returns a plot of the ground truth and reconstructed trajectories, a plot of the trajectory and orientation errors, and the values of the follwing performance metrics: RMSE, RMSE relative to path length, mean orientation error relative to path length. 
To run the odometry node and evaluate its performance, run the following commands from terminal: 
~~~
rosrun vision_based_nav VOnode.py
rosrun vision_based_nav LocalizationTest.py
~~~

>[!CAUTION]
>The simulation exploits the LEO rover model, which is included in the [ERC Leo rover repository](https://github.com/EuropeanRoverChallenge/ERC-Remote-Navigation-Sim). Clone this repository to download all the files needed to use the model. 

### Launcher 
for simplicity, all these commands can also be executed thanks to [Launcher.py](vision_based_nav/scripts/Launcher.py), a simple GUI which enables starting the simulation, running [VOnode.py](vision_based_nav/scripts/VOnode.py), running [LocalizationTest.py](vision_based_nav/scripts/LocalizationTest.py), as well as [DensePointCloud.py](vision_based_nav/scripts/DensePointCloud.py), which simply publishes a dense PointCloud message which can be used for mapping applications. A rudimental 2D projection is also implemented which can serve as foundation for future implementation of a geometrically-derived costmap.
To launch the GUI, run the following command from terminal (which should be sourced):  
~~~
rosrun vision_based_nav Launcher.py
~~~

# Expected Results
We report some examples of reconstructed trajectories obtained with the provided codes.

## Results on Katwijk dataset

![Position and orientation errors.](<vision_based_nav/figures/VOerrors_FAST_SGBM_kwjk.png>) 
![Reconstructed trajectory.](<vision_based_nav/figures/VO_FAST_SGBM_kwjk.png>)

## Results in simulated environment

![Position and orientation errors.](<vision_based_nav/figures/BRISK SGBM error.png>) 
![Reconstructed trajectory.](<vision_based_nav/figures/BRISK SGBM Trajectory.png>)

# Authors 

Ludovica Cavalieri, Mohamed El Awag, Brian Piccione
