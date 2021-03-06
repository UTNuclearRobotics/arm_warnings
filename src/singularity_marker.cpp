
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
  //The 'jointStateCB' callback function is called when new joint states are recieved 
  //From there current state and future predictions are analyzed
  joint_state_sub_ = n_.subscribe("/joint_states", 1, &singularity_marker::jointStateCB, this);

  // Get private parameters
  // Note: MoveGroup doesn't like namespaces, so each instance of this node needs a unique node name
  ros::NodeHandle pn("~");
  pn.param("singularity_threshold", singularity_threshold_, 60.0);
  pn.param("warning_threshold", warning_threshold_, 40.0);
  pn.getParam("move_group_name", move_group_name_);
  pn.getParam("ee_tf_name", ee_tf_name_);


  //Set up move_group and MoveIt! requirements
  moveit::planning_interface::MoveGroupInterface move_group(move_group_name_);

  robot_model_loader::RobotModelLoader model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = model_loader.getModel();
  
  joint_model_group_ = kinematic_model->getJointModelGroup(move_group_name_);
  kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model);

  joint_names_ =  joint_model_group_->getVariableNames();

  // Create the marker for RVIZ. 
  /* Singularity_Marker currently uses 'sin_marker_' as its only marker so 
  parameter will be reset and adjusted through out the program. This is ok
  because each marker displayed has a unique ID. In the future there should
  be consideration in making multiple custom markers for different aspect
  of singularity warning, prediction, and teleop aid. The is especially
  important if custom meshes / different shapes are going to be used. */

  sin_marker_.id = 0;
  sin_marker_.type = visualization_msgs::Marker::SPHERE;
  sin_marker_.action = visualization_msgs::Marker::ADD;
  sin_marker_.header.frame_id = ee_tf_name_;
  sin_marker_.scale.x = 0.4;
  sin_marker_.scale.y = 0.4;
  sin_marker_.scale.z = 0.4;
  sin_marker_.lifetime = ros::Duration(0.2);
  //Since marker will be displayed at EE of arm the frame id is set to the EE transform frame name
  sin_marker_.header.frame_id = ee_tf_name_;
  sin_marker_.pose.orientation.x = 0.0;
  sin_marker_.pose.orientation.y = 0.0;
  sin_marker_.pose.orientation.z = 0.0;
  sin_marker_.pose.orientation.w = 1.0;

  // Point toward singularity
  sin_direction_.id = 47;
  sin_direction_.type = visualization_msgs::Marker::ARROW;
  sin_direction_.action = visualization_msgs::Marker::ADD;
  // TODO: don't hard-code this
  sin_direction_.header.frame_id = "panda_link0";
  sin_direction_.lifetime = ros::Duration(0.2);
  sin_direction_.scale.x = 0.04;
  sin_direction_.scale.y = 0.1;
  sin_direction_.scale.z = 0.1;
  sin_direction_.color.r = 1.0f;
  sin_direction_.color.g = 0.0f;
  sin_direction_.color.b = 0.0f;
  sin_direction_.color.a = 1.0;

  // Point toward singularity
  eigenvector_.id = 48;
  eigenvector_.type = visualization_msgs::Marker::ARROW;
  eigenvector_.action = visualization_msgs::Marker::ADD;
  // TODO: don't hard-code this
  eigenvector_.header.frame_id = "panda_link0";
  eigenvector_.lifetime = ros::Duration(0.2);
  eigenvector_.scale.x = 0.04;
  eigenvector_.scale.y = 0.1;
  eigenvector_.scale.z = 0.1;
  eigenvector_.color.r = 0.0f;
  eigenvector_.color.g = 1.0f;
  eigenvector_.color.b = 0.0f;
  eigenvector_.color.a = 1.0;

  marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  //PUBLISHERS ARE USED FOR PROTOTYPING AND TESTING. Consider keeping or make unique message
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
  /* Extract the JOINT STATE

  A series of checks are made first to determine if the joint state is valid.
  If valid the a jacobian is requested and the condition of the jacobian is
  calculated. If the condition number exceed the warning_threshold then there
  will be a request for RVIZ markers. */

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
  
  //Get Jacobian
  kinematic_state_->setVariableValues(group_joints);
  Eigen::MatrixXd jacobian = kinematic_state_->getJacobian(joint_model_group_);

  std_msgs::Float32 condition_number;
  condition_number.data = checkConditionNumber(jacobian);
  condition_pub_.publish(condition_number);

  // Plot an arrow along the smallest singular vector (towards singularity)
  plotSVDDirection(jacobian);
  // Plot the eigenvector associated with smallest eigenvalue
  //plotEigenvector(jacobian);

  //Check if condition exceeds warning threshold
  ROS_ERROR_STREAM( "CN: " << condition_number );
  if (condition_number.data > warning_threshold_)
    //create the singularity markers. Functions requires the joint state for further calculation
    createMarkers(group_joints);
}

// Plot an arrow along the smallest eigenvector (towards singularity).
// By smallest eigenvector, I mean the one associated with smallest eigenvalue.
void arm_warnings::singularity_marker::plotEigenvector(const Eigen::MatrixXd &jacobian)
{
  //////////////////////////////////////////////
  // Need to trim the matrix down to 6x6.
  // Eliminate the column of smallest magnitude.
  //////////////////////////////////////////////
  std::vector<double> col_mags;

  for (int i=0; i<jacobian.cols(); ++i)
  {
    double magnitude = 0;
    for (int j=0; j<jacobian.rows(); ++j)
    {
      magnitude += jacobian(j,i)*jacobian(j,i);
    }
    col_mags.push_back( pow(magnitude,0.5) );
  }
  // Find the index of smallest magnitude
  const int N = sizeof(col_mags) / sizeof(double);
  const int smallest_col_index = std::min_element(col_mags.begin(), col_mags.end())-col_mags.begin();

  // Recreate a new matrix, minus this column
  Eigen::MatrixXd trimmed_jacobian(6,6);
  for ( int i=0; i<6; ++i )
    if ( i != smallest_col_index )
      trimmed_jacobian.col(i) = jacobian.col(i);
    else
      break;
  for ( int i=smallest_col_index; i<6; ++i)
    trimmed_jacobian.col(i) = jacobian.col(i);

  ///////////////////////////////////////////////////////
  // Calculate vector associated with smallest eigenvalue
  ///////////////////////////////////////////////////////
  Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(trimmed_jacobian);
  // Real components only
  Eigen::VectorXd eigenvector = eigen_solver.eigenvectors().col(5).real();

  //////////////////
  // Plot the vector
  //////////////////
  // I'm using this vector to convert an Eigen::Scalar to double. Prob better ways to do it.
  std::vector<double> vec;
  vec.push_back(eigenvector(0));
  vec.push_back(eigenvector(1));
  vec.push_back(eigenvector(2));

  // start pt = all zeros
  geometry_msgs::Point start_point;
  geometry_msgs::Point end_point;
  end_point.x=eigenvector[0];
  end_point.y=eigenvector[1];
  end_point.z=eigenvector[2];

  eigenvector_.points.clear();
  eigenvector_.points.push_back(start_point);
  eigenvector_.points.push_back(end_point);
  eigenvector_.header.stamp = ros::Time::now();

  marker_pub_.publish(eigenvector_);   

}

// Plot an arrow along the smallest singular vector (towards singularity)
void arm_warnings::singularity_marker::plotSVDDirection(Eigen::MatrixXd jacobian)
{
  // Find the direction away from nearest singularity.
  // The last column of U from the SVD of the Jacobian points away from the singularity
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU);
  Eigen::VectorXd vector_away_from_singularity = svd.matrixU().col(5);
  svd = Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinV);
  //ROS_WARN_STREAM( svd.matrixV().col(5) );

  // I'm using this vector to convert an Eigen::Scalar to double. Prob better ways to do it.
  std::vector<double> vec;
  vec.push_back(vector_away_from_singularity(0));
  vec.push_back(vector_away_from_singularity(1));
  vec.push_back(vector_away_from_singularity(2));
  vec.push_back(vector_away_from_singularity(3));
  vec.push_back(vector_away_from_singularity(4));
  vec.push_back(vector_away_from_singularity(5));

  // This singular vector tends to flip direction unpredictably. See R. Bro, "Resolving the Sign Ambiguity
  // in the Singular Value Decomposition"
  // "Look ahead" to see if the Jacobian's condition will decrease in this direction.
  // First, record the earlier condition number:
  double prev_condition = checkConditionNumber( jacobian );
  // scaled version of the singular vector
  Eigen::VectorXd delta_x(6);
  double scale = 100;
  delta_x[0] = vec[0]/scale;
  delta_x[1] = vec[1]/scale;
  delta_x[2] = vec[2]/scale;
  delta_x[3] = vec[3]/scale;
  delta_x[4] = vec[4]/scale;
  delta_x[5] = vec[5]/scale;

  // Calculate a small change in joints
  Eigen::VectorXd delta_theta = pseudoInverse( jacobian ) * delta_x;
  double theta [6];
  const double* prev_joints = kinematic_state_->getVariablePositions();
  for (std::size_t i = 0, size = static_cast<std::size_t>(delta_theta.size()); i < size; ++i)
  {
    try
    {
      theta[i] = prev_joints[i]+delta_theta(i);
    }
    catch (const std::out_of_range& e)
    {
      ROS_ERROR_STREAM(" Lengths of output and increments do not match.");
    }
  }

  kinematic_state_->setVariablePositions( theta );
  jacobian = kinematic_state_->getJacobian(joint_model_group_);
  double new_condition = checkConditionNumber( jacobian );

  // If new_condition < prev_condition, the singular vector does point towards a singularity.
  //  Otherwise, flip its direction.
  if ( prev_condition >= new_condition )
    std::transform(vec.cbegin(),vec.cend(),vec.begin(),std::negate<double>());

  // start pt = all zeros
  geometry_msgs::Point start_point;
  geometry_msgs::Point end_point;
  end_point.x=vec[0];
  end_point.y=vec[1];
  end_point.z=vec[2];

  sin_direction_.points.clear();
  sin_direction_.points.push_back(start_point);
  sin_direction_.points.push_back(end_point);
  sin_direction_.header.stamp = ros::Time::now();

  marker_pub_.publish(sin_direction_); 
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
  // This is a red sphere centered on end effector

  sin_marker_.id = 0;
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

  //create prediction markers along axes
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
  //set size of 0.01. This is the size of arm steps used for calculations.
  float scale = 0.01;
  //prepare twist to calculate delta_x. no rotation values (yet?)
  geometry_msgs::TwistStamped twist_cmd;
  twist_cmd.twist.angular.x = 0;
  twist_cmd.twist.angular.y = 0;
  twist_cmd.twist.angular.z = 0;

  //Do calculations and create marker for each direction
  for (int i = 0; i < 6; i++)
  {
    twist_cmd.twist.linear.x = dir[i][0] * scale;
    twist_cmd.twist.linear.y = dir[i][1] * scale;
    twist_cmd.twist.linear.z = dir[i][2] * scale;
    //send desired direction, joint state, and number steps to predictCondition to received jacobian
    Eigen::MatrixXd jacobian = predictCondition(twist_cmd, group_joints, 3);
    condition_list[i] = checkConditionNumber(jacobian);

    sin_marker_.id = i + 1;
    //Custom marker meshes would go HERE
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
  //const Eigen::VectorXd delta_x = scaleCommand(twist_cmd);

  //Turn twist in to the 'delta_x' vector
  Eigen::VectorXd delta_x(6);

  delta_x[0] = twist_cmd.twist.linear.x;
  delta_x[1] = twist_cmd.twist.linear.y;
  delta_x[2] = twist_cmd.twist.linear.z;
  delta_x[3] = twist_cmd.twist.angular.x;
  delta_x[4] = twist_cmd.twist.angular.y;
  delta_x[5] = twist_cmd.twist.angular.z;

  //Do calculations for requested number of steps
  for (int i = 0; i < steps; i++)
  {

    kinematic_state_->setVariableValues(group_joints);
    Eigen::MatrixXd jacobian = kinematic_state_->getJacobian(joint_model_group_);
    Eigen::VectorXd delta_theta = pseudoInverse(jacobian) * delta_x;

    //add delta_theta to the joint_state so calculations can be repeated
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

  ///////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////

  // This if statements is used for testing! Returns results for the positive x direction
  // Trying to determine for the effect of number of steps and step size

  if (twist_cmd.twist.linear.x > 0)
    {
      kinematic_state_->setVariableValues(group_joints);
      Eigen::MatrixXd test_jacobian = kinematic_state_->getJacobian(joint_model_group_);

      std_msgs::Float32 condition_debug;
      condition_debug.data = checkConditionNumber(test_jacobian);
      new_condition_pub_.publish(condition_debug);
    }
  }

  /////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////

  //Return Jacobian
  kinematic_state_->setVariableValues(group_joints);
  return kinematic_state_->getJacobian(joint_model_group_);

}

