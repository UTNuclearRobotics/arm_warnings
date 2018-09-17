
///////////////////////////////////////////////////////////////////////////////
//      Title     : singularity_marker.cpp
//      Project   : Teleop IK
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


arm_warnings::singularity_marker::singularity_marker()
{
  joint_state_sub_ = n_.subscribe("/joint_states", 1, &singularity_marker::jointStateCB, this);

  // Get private parameters
  // Note: MoveGroup doesn't like namespaces, so each instance of this node needs a unique node name
  ros::NodeHandle pn("~");
  pn.param("singularity_threshold", singularity_threshold_, 60.0);
  pn.param("warning_threshold", warning_threshold_, 40.0);
  pn.getParam("marker_id", marker_id_);
  pn.getParam("ee_tf_name", ee_tf_name_);
  //!!! FOR TESTING PURPOSES I EXPLICITLY DEFINE THE MOVE GROUP NAME, I NEED A CONFIG FILE!!!!!
  move_group_name_ = "panda_arm";
  

  moveit::planning_interface::MoveGroupInterface move_group(move_group_name_);

  robot_model_loader::RobotModelLoader model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = model_loader.getModel();
  
  joint_model_group_ = kinematic_model->getJointModelGroup(move_group_name_);
  kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model);

  //joint_names_ = move_group_->getJointNames();
  joint_names_ =  joint_model_group_->getVariableNames();

  // This marker indicates the joint that has an error. It's a red sphere
  sin_marker_.id = marker_id_;
  sin_marker_.type = visualization_msgs::Marker::SPHERE;
  sin_marker_.action = visualization_msgs::Marker::ADD;
  sin_marker_.header.frame_id = ee_tf_name_;

  sin_marker_.scale.x = 0.4;
  sin_marker_.scale.y = 0.4;
  sin_marker_.scale.z = 0.4;

  sin_marker_.lifetime = ros::Duration(0.2);

  //AGAIN NEED YAML for ee_name
  sin_marker_.header.frame_id = "panda_hand";

  sin_marker_.pose.orientation.x = 0.0;
  sin_marker_.pose.orientation.y = 0.0;
  sin_marker_.pose.orientation.z = 0.0;
  sin_marker_.pose.orientation.w = 1.0;

  marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  //PUBLISHERS ARE USED FOR PROTOTYPING
  condition_pub_ = n_.advertise<std_msgs::Float32>("arm_condition", 1);
  new_condition_pub_ = n_.advertise<std_msgs::Float32>("arm_condition_pseudo", 1);

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
    ROS_ERROR_STREAM_THROTTLE(2, "JOINT MESSAGE EMPTY");
    return;
  }
  const sensor_msgs::JointState group_joints = extractMyJointInfo(msg);
  
  if (group_joints.name.empty()) {
    ROS_ERROR_STREAM_THROTTLE(2, "no joint group name");
    return;
  }
  
  kinematic_state_->setVariableValues(group_joints);
  Eigen::MatrixXd jacobian = kinematic_state_->getJacobian(joint_model_group_);

  std_msgs::Float32 condition_number;
  condition_number.data = checkConditionNumber(jacobian);
  condition_pub_.publish(condition_number);
  if (condition_number.data > warning_threshold_)
    createMarkers(group_joints);

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

void arm_warnings::singularity_marker::createMarkers(sensor_msgs::JointState group_joints)
{
  // Publish general singularity marker in RVIZ
  sin_marker_.id = 1;
  sin_marker_.pose.position.x = 0;
  sin_marker_.pose.position.y = 0;
  sin_marker_.pose.position.z = 0;
  sin_marker_.scale.x = 0.15;
  sin_marker_.scale.y = 0.15;
  sin_marker_.scale.z = 0.15;
  sin_marker_.color.r = 1.0f;
  sin_marker_.color.g = 0.0f;
  sin_marker_.color.b = 0.0f;
  sin_marker_.color.a = 1.0;
  sin_marker_.header.stamp = ros::Time::now();
  marker_pub_.publish(sin_marker_);

  axesMarkers(group_joints);
}

void arm_warnings::singularity_marker::axesMarkers(sensor_msgs::JointState group_joints)
{
  // +/- in along 3D axes
  int dir [6][3] = 
  {{1, 0, 0},
  {-1, 0, 0},
  {0, 1, 0},
  {0, -1, 0},
  {0, 0, 1},
  {0, 0, -1},
  };
  float condition_list [6];
  //set size of 0.01,
  float scale = 0.01;
  //prepare twist to calculate delta_x. no rotation values (yet?)
  geometry_msgs::TwistStamped twist_cmd;
  twist_cmd.twist.angular.x = 0;
  twist_cmd.twist.angular.y = 0;
  twist_cmd.twist.angular.z = 0;


  for (int i = 0; i < 6; i++)
  {
    
    twist_cmd.twist.linear.x = dir[i][0] * scale;
    twist_cmd.twist.linear.y = dir[i][1] * scale;
    twist_cmd.twist.linear.z = dir[i][2] * scale;
    
    Eigen::MatrixXd jacobian = predictCondition(twist_cmd, group_joints, 3);
    
    condition_list[i] = checkConditionNumber(jacobian);

    //std_msgs::Float32 condition_number;
    //condition_number.data = checkConditionNumber(jacobian);
    //new_condition_pub_.publish(condition_number);

    sin_marker_.id = i + 2;
    //sin_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    //sin_marker_.mesh_resource = "package://arm_warnings/meshes/custom_marker.dae";

    sin_marker_.pose.position.x = dir[i][0] * scale * 10;
    sin_marker_.pose.position.y = dir[i][1] * scale * 10;
    sin_marker_.pose.position.z = dir[i][2] * scale * 10;

    sin_marker_.scale.x = 0.075;
    sin_marker_.scale.y = 0.075;
    sin_marker_.scale.z = 0.075;

    sin_marker_.color.r = 0.0f;
    sin_marker_.color.g = 0.0f;
    sin_marker_.color.b = 0.0f;
    sin_marker_.color.a = 1.0;

    if (condition_list[i] < 30)
    {
     sin_marker_.color.g = 1.0f; 
    }
    else if (condition_list[i] < 50)
    {
      sin_marker_.color.b = 1.0f;
    }
    else
    {
      sin_marker_.color.r = 1.0f;
    }
    

    marker_pub_.publish(sin_marker_);
  }
}


Eigen::MatrixXd arm_warnings::singularity_marker::predictCondition(geometry_msgs::TwistStamped twist_cmd, sensor_msgs::JointState group_joints, int steps)
{
  const Eigen::VectorXd delta_x = scaleCommand(twist_cmd);
  for (int i = 0; i < steps; i++)
  {

    kinematic_state_->setVariableValues(group_joints);
    Eigen::MatrixXd jacobian = kinematic_state_->getJacobian(joint_model_group_);
    Eigen::VectorXd delta_theta = pseudoInverse(jacobian) * delta_x;

    for (std::size_t i = 0, size = static_cast<std::size_t>(delta_theta.size()); i < size; ++i)
    {
      
      try
      {
        group_joints.position[i] += delta_theta[static_cast<long>(i)];
      }
      catch (const std::out_of_range& e)
      {
        ROS_ERROR_STREAM_THROTTLE(2, "failed to add delta theta bro");
      }
    }


  if (twist_cmd.twist.linear.x > 0)
    {
      kinematic_state_->setVariableValues(group_joints);
      Eigen::MatrixXd test_jacobian = kinematic_state_->getJacobian(joint_model_group_);

      std_msgs::Float32 condition_debug;
      condition_debug.data = checkConditionNumber(test_jacobian);
      new_condition_pub_.publish(condition_debug);
    }
  }

  kinematic_state_->setVariableValues(group_joints);
  return kinematic_state_->getJacobian(joint_model_group_);

}

Eigen::VectorXd arm_warnings::singularity_marker::scaleCommand(const geometry_msgs::TwistStamped& command) const
{
  Eigen::VectorXd result(6);

  result[0] = command.twist.linear.x;
  result[1] = command.twist.linear.y;
  result[2] = command.twist.linear.z;
  result[3] = command.twist.angular.x;
  result[4] = command.twist.angular.y;
  result[5] = command.twist.angular.z;

  return result;
}