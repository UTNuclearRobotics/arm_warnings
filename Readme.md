#arm_warnings

This package is used for visualizing and predicting undesirable joint states.

arm_warnings creates RVIZ markers that can be useful for teleoperation, 
developing, and testing arms.

Singularity Marker predicts the condition of future joint states. While aimed to
be used with any ROS compatable arm, the launch and config file are taylored to 
be used with MoveIt panda arm tutoral simulation. To run Singularity Marker and 
the panda sim...

roslaunch panda_moveit_config demo.launch

roslaunch arm_warnings panda_sim.launch