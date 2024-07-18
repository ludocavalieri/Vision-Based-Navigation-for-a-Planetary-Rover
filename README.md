# Vision-based Navigation for a Planetary Rover

This repository, articulated in two branches, contains the Colab notebook and ROS package used to perform visual odometry with the ESA Katwijck beach dataset and in a simulated environment, respectively.

# Abstract

Communication and computing performances of planetary rovers limit their autonomous traverse distances. Moreover, for missions to Mars, manual driving is unfeasible: not only does the round trip communication time between Earth and Mars takes from 5 to 20 minutes, but contact with ground can be reached only twice per Sol. Even with the help of orbital data, planning long traverses, and avoiding all local obstacles the rover may encounter on its path, proves to be nearly an impossible task. Such rovers are expensive and out of reach for physical help, and therefore, any collision critically endangers the mission's success. Neverthless, considering future missions' programs, there is the necessity for a high degree of autonomy of these robotic assets.

This work aims to develop an algorithm for autonomous localization and mapping of a planetary rover employing the images provided by a stereocamera. The focus of the study will be on the computer vision techniques that are able to provide useful information for the navigation task, but the particular topic of the rover guidance will not be tackled. In the context of our specific application, great attention will be given to the efficiency of the proposed algorithm and its ability to run in real time.

In particular, the rover trajectory and pose will be reconstructed using Visual Odometry. Then, an obstacle detection algorithm will be employed to identify and locate possible hazardous objects in the rover frame. In this context, different O.D. techniques will be surveyed in order to test their performances. 

The ultimate output of the pipeline will be a medium-to-close range obstacle map reconstructed entirely from stereo camera data. The overall performances will be evaluated comparing the ground truth trajectory and obstacle map with the reconstructed ones.

# Running the code

## Visual odometry with Katwijck dataset

## Visual odometry in simulated environment

### Start the simulation
The simulation environment is specified in [model.sdf](models/mars_like_environment/model.sdf), where it is possible to select *katwijck simulated.dae* amd *model.dae*. The former is a basic reproduction of the Katwijck beach portion in which the real dataset is acquired, while the latter is a slightly more complex and realistic reproduction of a planetary terrain. The node [RobotController.py](scripts/RobotController.py) enables sending */cmd_vel* messages to the rover using the 
To launch the simulation environment (both Gazebo and Rviz), run the following commands from terminal: 
'''
roslaunch vision_based_nav gazebo_model.launch 
rosrun vision_based_nav RobotController.py
'''
>[!WARNING]
>To maintain the controller active, click on the GUI.

### Perform visual odometry and evaluate its performance 
The Visual Odometry procedure is implemented in [VOnode.py](scripts/VOnode.py). To test the pipeline's performance, run [LocalizationTest.py](scripts/LocalizationTest.py), which returns a plot of the ground truth and reconstructed trajectories, a plot of the trajectory and orientation errors, and the values of the following metrics: RMSE, RMSE relative to path length, mean orientation error relative to path length. 
To run the odometry node and evaluate its performance, run the following commands from terminal: 
'''
rosrun vision_based_nav VOnode.py
rosrun vision_based_nav LocalizationTest.py
'''

>[!CAUTION]
>The simulation exploits the leo rover model. To download the leo rover repository head to [ERC Leo rover repository](https://github.com/EuropeanRoverChallenge/ERC-Remote-Navigation-Sim).

# Authors 

Ludovica Cavalieri, Mohamed El Awag, Brian Piccione