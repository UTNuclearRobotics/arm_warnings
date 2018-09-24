#ifndef arm_warning_H
#define arm_warning_H

#include <eigen3/Eigen/Eigenvalues>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sound_play/sound_play.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>


namespace arm_warnings
{

class singularity_marker
{
public:
  singularity_marker();

private:
  void jointStateCB(sensor_msgs::JointStateConstPtr msg);
  double checkConditionNumber(Eigen::MatrixXd& matrix) const;
  void createMarkers(sensor_msgs::JointState group_joints);
  void axesMarkers(sensor_msgs::JointState group_joints);
  Eigen::MatrixXd predictCondition(geometry_msgs::TwistStamped twist_cmd, sensor_msgs::JointState group_joints, int steps);
  Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& J) const;

  // Plot an arrow along the smallest eigenvector (towards singularity).
  // By smallest eigenvector, I mean the one associated with smallest eigenvalue.
  void plotSVDDirection(const Eigen::MatrixXd &jacobian);

  // Plot an arrow along the smallest eigenvector (towards singularity)
  void plotEigenvector(const Eigen::MatrixXd &jacobian);

  const sensor_msgs::JointState extractMyJointInfo(sensor_msgs::JointStateConstPtr original) const;

///////////////////////////////

  double singularity_threshold_, warning_threshold_;

  const robot_state::JointModelGroup* joint_model_group_;

  std::vector<std::string> joint_names_;

  ros::Subscriber joint_state_sub_;

  robot_state::RobotStatePtr kinematic_state_;

  std::string move_group_name_;

  ros::NodeHandle n_;

  visualization_msgs::Marker sin_marker_, sin_direction_, eigenvector_;

  ros::Publisher marker_pub_, condition_pub_, new_condition_pub_;

  int marker_id_;

  std::string ee_tf_name_;
};

} // End arm_warnings namespace

#endif
