
///////////////////////////////////////////////////////////////////////////////
//      Title     : singularity_marker.cpp
//      Project   : vaultbot_nrg
//      Created   : 9/3/18
//      Author    : Conner Dimoush
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2014-2017. All rights reserved.
//                 
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////


#include "singularity_marker.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "singularity_marker");

  arm_warnings::singularity_marker s_mark;

  return 0;
}


arm_warnings::singularity_marker::singularity_marker() :
  move_group_(NULL)
{
  joint_state_sub_ = n_.subscribe("/joint_states", 1, &singularity_marker::jointStateCB, this);

  // Get private parameters
  // Note: MoveGroup doesn't like namespaces, so each instance of this node needs a unique node name
  ros::NodeHandle pn("~");
  pn.param("singularity_threshold", singularity_threshold_, 50.0);
  pn.getParam("marker_id", marker_id_);
  //should be somethink arm
  pn.getParam("move_group_name", move_group_name_ );
  pn.getParam("ee_tf_name", ee_tf_name_);

  if (move_group_ != NULL) {
    delete move_group_;
  }
  
  move_group_ = new moveit::planning_interface::MoveGroup(move_group_name_);

  robot_model_loader::RobotModelLoader model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = model_loader.getModel();
  
  joint_model_group_ = kinematic_model->getJointModelGroup(move_group_name_);
  kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model);

  joint_names_ = move_group_->getJointNames();

  // This marker indicates the joint that has an error. It's a red sphere
  sin_marker_.id = marker_id_;
  sin_marker_.type = visualization_msgs::Marker::SPHERE;
  sin_marker_.action = visualization_msgs::Marker::ADD;
  sin_marker_.header.frame_id = ee_tf_name_;

  sin_marker_.scale.x = 0.2;
  sin_marker_.scale.y = 0.2;
  sin_marker_.scale.z = 0.2;

  sin_marker_.color.r = 1.0f;
  sin_marker_.color.g = 0.0f;
  sin_marker_.color.b = 0.0f;
  sin_marker_.color.a = 1.0;
  sin_marker_.lifetime = ros::Duration(0.2);

  sin_marker_.pose.position.x = 0;
  sin_marker_.pose.position.y = 0;
  sin_marker_.pose.position.z = 0;
  sin_marker_.pose.orientation.x = 0.0;
  sin_marker_.pose.orientation.y = 0.0;
  sin_marker_.pose.orientation.z = 0.0;
  sin_marker_.pose.orientation.w = 1.0;

  marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  while ( ros::ok() )
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
}

void arm_warnings::singularity_marker::jointStateCB(sensor_msgs::JointStateConstPtr msg)
{
  // Check that the msg contains joints
  if (msg->name.empty()) {
    return;
  }
  const sensor_msgs::JointState group_joints = extractMyJointInfo(msg);
  
  if (group_joints.name.empty()) {
    return;
  }
  
  kinematic_state_->setVariableValues(group_joints);
  Eigen::MatrixXd jacobian = kinematic_state_->getJacobian(joint_model_group_);

 
  
  if (checkConditionNumber(jacobian) > singularity_threshold_)
    createMarkers();

}

const sensor_msgs::JointState arm_warnings::singularity_marker::extractMyJointInfo(sensor_msgs::JointStateConstPtr original) const
{
  sensor_msgs::JointState my_joint_info;
  my_joint_info.header = original->header;

  for (unsigned int i = 0, size = original->name.size(); i < size; ++i)
  {
    std::string name = original->name.at(i);
    if (std::find(joint_names_.begin(), joint_names_.end(), name) != joint_names_.end()) {
      my_joint_info.name.push_back(name);
      my_joint_info.position.push_back(original->position.at(i));
    }
  }
  
  return my_joint_info;  
}

double arm_warnings::singularity_marker::checkConditionNumber(Eigen::MatrixXd& matrix) const
{
  return pseudoInverse(matrix).norm() * matrix.norm();
}

Eigen::MatrixXd arm_warnings::singularity_marker::pseudoInverse(const Eigen::MatrixXd& J) const
{
  return J.transpose() * (J * J.transpose()).inverse();
}

void arm_warnings::singularity_marker::createMarkers()
{

}