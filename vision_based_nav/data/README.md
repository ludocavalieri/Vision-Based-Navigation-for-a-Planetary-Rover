## Reading data from rosbag vs. running the simulation: 

The file rosbagterraincustom.bag contains recorded messages published by the gazebo simulation on the /cmd_vel, /tf and /tf_static topic. It can be played from the data folder using the command rosbag play rosbagcustomterrain.bag. 

! The /tf topic only work if the following line is executed from terminal: 
  rosparam set use_sim_time true

The rosbag provides a reference trajectory, which can be useful when running tests (e.g. comparing the performance of the different detectors).