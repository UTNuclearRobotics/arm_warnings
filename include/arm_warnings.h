#ifndef arm_warning_H
#define arm_warning_H

#include <eigen3/Eigen/Eigenvalues>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sound_play/sound_play.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

namespace arm_warnings
{

class arm_warnings
{
public:
  arm_warnings();

private:
  void jointStateCB(sensor_msgs::JointStateConstPtr msg);

  const sensor_msgs::JointState extractMyJointInfo(sensor_msgs::JointStateConstPtr original) const;

  //bool checkConditionNumber(const Eigen::MatrixXd &matrix, double threshold) const;

  //void throwSingularityAlarm();

  enum joint_limit_status { OK = 0, LOW_LIMIT_WARNING = 1, LOW_LIMIT_ERROR = 2, HIGH_LIMIT_WARNING = 3, HIGH_LIMIT_ERROR = 4 };
  joint_limit_status checkJointLimits(const sensor_msgs::JointState &group_joints);

  void throwJointLimitWarning(joint_limit_status status);
  void throwJointLimitError(joint_limit_status status);

  // Create a paired list of joint & tf names so we can get the joint locations
  void create_jt_tf_list(std::string prefix);

///////////////////////////////

  double cond_number_threshold_, rad_min_, rad_max_;

  const robot_state::JointModelGroup* joint_model_group_;

  std::vector<std::string> joint_names_;

  ros::Subscriber joint_state_sub_;

  robot_state::RobotStatePtr kinematic_state_;

  moveit::planning_interface::MoveGroupInterface* move_group_;

  std::string move_group_name_, troublesome_jt_;

  ros::NodeHandle n_;

  visualization_msgs::Marker jt_marker_;

  ros::Publisher marker_pub_;

  int marker_id_;

  std::vector< std::pair<std::string, std::string> > jt_tf_names_;
};

} // End arm_warnings namespace

#endif
