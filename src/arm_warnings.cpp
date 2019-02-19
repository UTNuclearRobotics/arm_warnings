
///////////////////////////////////////////////////////////////////////////////
//      Title     : arm_warnings.cpp
//      Project   : vaultbot_nrg
//      Created   : 2/22/2017
//      Author    : Blake Anderson, Andy Zelenak
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

// Some checking for common errors, sometimes giving audible warnings.

#include "arm_warnings.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_warnings");

  arm_warnings::arm_warnings warn;

  return 0;
}

arm_warnings::arm_warnings::arm_warnings() :
  move_group_(NULL)
{
  joint_state_sub_ = n_.subscribe("/joint_states", 1, &arm_warnings::jointStateCB, this);

  // Get private parameters
  // Note: MoveGroup doesn't like namespaces, so each instance of this node needs a unique node name
  ros::NodeHandle pn("~");
  //pn.param("condition_threshold", cond_number_threshold_, 50.0);
  pn.getParam("minimum_joint_rad", rad_min_);
  pn.getParam("maximum_joint_rad", rad_max_);
  pn.getParam("marker_id", marker_id_);
  pn.getParam("move_group_name", move_group_name_ );

  if (move_group_ != NULL) {
    delete move_group_;
  }
  
  move_group_ = new moveit::planning_interface::MoveGroupInterface(move_group_name_);

  robot_model_loader::RobotModelLoader model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = model_loader.getModel();
  
  joint_model_group_ = kinematic_model->getJointModelGroup(move_group_name_);
  kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model);

  joint_names_ = move_group_->getJointNames();

  // This marker indicates the joint that has an error. It's a red sphere
  jt_marker_.id = marker_id_;
  jt_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  jt_marker_.mesh_resource = "package://arm_warnings/meshes/arrow.STL";
  jt_marker_.action = visualization_msgs::Marker::ADD;

  jt_marker_.scale.x = 1;
  jt_marker_.scale.y = 1;
  jt_marker_.scale.z = 1;

  jt_marker_.color.r = 1.0f;
  jt_marker_.color.g = 0.0f;
  jt_marker_.color.b = 0.0f;
  jt_marker_.color.a = 1.0;
  jt_marker_.lifetime = ros::Duration(0.5);

  jt_marker_.pose.position.x = 0;
  jt_marker_.pose.position.y = 0;
  jt_marker_.pose.position.z = 0;
  jt_marker_.pose.orientation.x = 0.0;
  jt_marker_.pose.orientation.y = 0.0;
  jt_marker_.pose.orientation.z = 0.0;
  jt_marker_.pose.orientation.w = 1.0;

  marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Create a paired list of joints/tf's so we know each joint's location in space
  create_jt_tf_list("right_ur5_");
  create_jt_tf_list("left_ur5_");

  while ( ros::ok() )
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
}

void arm_warnings::arm_warnings::jointStateCB(sensor_msgs::JointStateConstPtr msg)
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

  arm_warnings::arm_warnings::joint_limit_status status = checkJointLimits(group_joints);

  if ( status == joint_limit_status::LOW_LIMIT_ERROR || status == joint_limit_status::HIGH_LIMIT_ERROR )
    throwJointLimitError(status);
  else if (  status == joint_limit_status::LOW_LIMIT_WARNING || status == joint_limit_status::HIGH_LIMIT_WARNING )
    throwJointLimitWarning(status);
}

const sensor_msgs::JointState arm_warnings::arm_warnings::extractMyJointInfo(sensor_msgs::JointStateConstPtr original) const
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

// TODO: this only finds one joint near the limit before returning. There could be multiple.
arm_warnings::arm_warnings::joint_limit_status arm_warnings::arm_warnings::checkJointLimits(const sensor_msgs::JointState &group_joints)
{

  if ( !group_joints.name.empty() )
  {
    // Check for an error first: limit exceeded
    for (int i=0; i<group_joints.position.size(); ++i)
    {
      if (group_joints.position.at(i)<rad_min_)
      {
      	troublesome_jt_ = std::string(group_joints.name[i]);
        return LOW_LIMIT_ERROR;
      }
      else if (group_joints.position.at(i)>rad_max_)
      {
        troublesome_jt_ = std::string(group_joints.name[i]);
        return HIGH_LIMIT_ERROR;
      }
    }

  // Now check for a warning: close to limit
	for (int i=0; i<group_joints.position.size(); ++i)
    {
      if ( group_joints.position.at(i) < (rad_min_ + 0.2) )
      {
      	troublesome_jt_ = std::string(group_joints.name[i]);
        return LOW_LIMIT_WARNING;
      }

      else if (group_joints.position.at(i) > (rad_max_ - 0.2))
      {
        troublesome_jt_ = std::string(group_joints.name[i]);
        return HIGH_LIMIT_WARNING;
      }
  	}
  }
   
  return OK;
}

void arm_warnings::arm_warnings::throwJointLimitWarning(arm_warnings::arm_warnings::joint_limit_status status)
{
  // Find the tf frame corresponding to the troublesome jt
  for (int i=0; i<jt_tf_names_.size(); ++i)
  {
  	if ( jt_tf_names_[i].first == troublesome_jt_ )
  	{
      //Frame id must match a tf
      jt_marker_.header.frame_id = jt_tf_names_[i].second;
      break;
  	}
  }

  // Flip the arrow depending on whether we're close to a low limit or a high limit.
  // (This assumes joint z-axis directions are consistent.)
  if (status == HIGH_LIMIT_WARNING)
  {
    tf2::Quaternion q_rot, q;
    q_rot.setRPY(0, 0, 3.14159);
    tf2::convert(jt_marker_.pose.orientation , q);
    q = q_rot * q;
    tf2::convert(q, jt_marker_.pose.orientation);
  }

  // Indicate the troublesome joint in RViz
  jt_marker_.color.r = 1.0f;
  jt_marker_.color.g = 0.5f;
  jt_marker_.color.b = 0.0f;
  jt_marker_.color.a = 1.0;
  jt_marker_.header.stamp = ros::Time::now();
  marker_pub_.publish(jt_marker_);
}

void arm_warnings::arm_warnings::throwJointLimitError(arm_warnings::arm_warnings::joint_limit_status status)
{
  ROS_ERROR_STREAM_THROTTLE(2, "Warning: Joint Limit");

  // Find the tf frame corresponding to the troublesome jt.
  // Put the arrow there.
  for (int i=0; i<jt_tf_names_.size(); ++i)
  {
    //Frame id must match a tf
  	if ( jt_tf_names_[i].first == troublesome_jt_ )
  	{
      jt_marker_.header.frame_id = jt_tf_names_[i].second;
      break;
  	}
  }

  // Flip the arrow depending on whether we're close to a low limit or a high limit.
  // (This assumes joint z-axis directions are consistent.)
  if (status == HIGH_LIMIT_ERROR)
  {
    tf2::Quaternion q_rot, q;
    q_rot.setRPY(0, 0, 3.14159);
    tf2::convert(jt_marker_.pose.orientation , q);
    q = q_rot * q;
    tf2::convert(q, jt_marker_.pose.orientation);
  }

  // Indicate the troublesome joint in RViz
  jt_marker_.color.r = 1.0f;
  jt_marker_.color.g = 0.0f;
  jt_marker_.color.b = 0.0f;
  jt_marker_.color.a = 1.0;
  jt_marker_.header.stamp = ros::Time::now();
  marker_pub_.publish(jt_marker_);
}

// Create a paired list of joint & tf names so we can get the joint locations
void arm_warnings::arm_warnings::create_jt_tf_list(std::string prefix)
{
  //
  std::pair<std::string, std::string> p;

  p.first = prefix + "shoulder_pan_joint"; p.second = prefix + "shoulder_link";
  jt_tf_names_.push_back(p);
  p.first = prefix + "shoulder_lift_joint"; p.second = prefix + "upper_arm_link";
  jt_tf_names_.push_back(p);
  p.first = prefix + "elbow_joint"; p.second = prefix + "forearm_link";
  jt_tf_names_.push_back(p);
  p.first = prefix + "wrist_1_joint"; p.second = prefix + "wrist_1_link";
  jt_tf_names_.push_back(p);
  p.first = prefix + "wrist_2_joint"; p.second = prefix + "wrist_2_link";
  jt_tf_names_.push_back(p);
  p.first = prefix + "wrist_3_joint"; p.second = prefix + "wrist_3_link";
  jt_tf_names_.push_back(p);
}
