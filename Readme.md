## arm_warnings

This package is used for visualizing and predicting undesirable joint states.

### arm warnings
arm_warnings creates RVIZ markers that can be useful for teleoperation, 
developing, and testing arms.

### singularity marker
Singularity Marker predicts the condition of future joint states. While aimed to
be used with any ROS compatable arm, the launch and config file are tailored to 
be used with MoveIt panda arm tutoral simulation. Copy the panda setup files from ./panda_stuff to the launch folder of the panda_moveit_config package. Then,

roslaunch panda_moveit_config demo.launch

roslaunch arm_warnings panda_sim.launch