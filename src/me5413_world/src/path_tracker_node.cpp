/** path_tracker_node.cpp
 *
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 *
 * MIT License
 *
 * ROS Node for robot to track a given path
 */

#include "me5413_world/math_utils.hpp"
#include "me5413_world/path_tracker_node.hpp"

namespace me5413_world
{

// Dynamic Parameters
double SPEED_TARGET;
double PID_Kp, PID_Ki, PID_Kd;
double STANLEY_K;
bool PARAMS_UPDATED;

// Zizia.
double LOOKAHEAD_DISTANCE = 1.5;

double Feedback_K1 = 1, Feedback_K2 = 10;
double Feedback_miu = 1, Feedback_lambda = 3;

void dynamicParamCallback(const me5413_world::path_trackerConfig& config, uint32_t level)
{
  // Common Params
  SPEED_TARGET = config.speed_target;
  // PID
  PID_Kp = config.PID_Kp;
  PID_Ki = config.PID_Ki;
  PID_Kd = config.PID_Kd;
  // Stanley
  STANLEY_K = config.stanley_K;

  PARAMS_UPDATED = true;
}

PathTrackerNode::PathTrackerNode() : tf2_listener_(tf2_buffer_)
{
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
  this->sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
  this->pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);

  // Initialization
  this->robot_frame_ = "base_link";
  this->world_frame_ = "world";

  this->pid_ = control::PID(0.1, 1.0, -1.0, PID_Kp, PID_Ki, PID_Kd);
}

void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
{
  // Calculate absolute errors (wrt to world frame)
  this->pose_world_goal_ = path->poses[11].pose;
  // this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_, this->pose_world_goal_));
  this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_, this->pose_world_goal_));

  return;
}

void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->odom_world_robot_ = *odom.get();

  return;
}

double PathTrackerNode::computeStanelyControl(const double heading_error, const double cross_track_error, const double velocity)
{
  const double stanley_output = -1.0*(heading_error + std::atan2(STANLEY_K*cross_track_error, std::max(velocity, 0.3)));

  return std::min(std::max(stanley_output, -2.2), 2.2);
}

geometry_msgs::Twist PathTrackerNode::computeControlOutputs(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal)
{
  // Heading Error
  tf2::Quaternion q_robot, q_goal;
  tf2::fromMsg(odom_robot.pose.pose.orientation, q_robot);
  tf2::fromMsg(pose_goal.orientation, q_goal);
  const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
  const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_goal);

  double roll, pitch, yaw_robot, yaw_goal;
  m_robot.getRPY(roll, pitch, yaw_robot);
  m_goal.getRPY(roll, pitch, yaw_goal);

  const double heading_error = unifyAngleRange(yaw_robot - yaw_goal);

  // Lateral Error
  tf2::Vector3 point_robot, point_goal;
  tf2::fromMsg(odom_robot.pose.pose.position, point_robot);
  tf2::fromMsg(pose_goal.position, point_goal);
  const tf2::Vector3 V_goal_robot = point_robot - point_goal;
  const double angle_goal_robot = std::atan2(V_goal_robot.getY(), V_goal_robot.getX());
  const double angle_diff = angle_goal_robot - yaw_goal;
  const double lat_error = V_goal_robot.length()*std::sin(angle_diff);

  // Velocity
  tf2::Vector3 robot_vel;
  tf2::fromMsg(this->odom_world_robot_.twist.twist.linear, robot_vel);
  const double velocity = robot_vel.length();

  geometry_msgs::Twist cmd_vel;
  if (PARAMS_UPDATED)
  {
    this->pid_.updateSettings(PID_Kp, PID_Ki, PID_Kd);
    PARAMS_UPDATED = false;
  }
  cmd_vel.linear.x = this->pid_.calculate(SPEED_TARGET, velocity);
  cmd_vel.angular.z = computeStanelyControl(heading_error, lat_error, velocity);

  // std::cout << "robot velocity is " << velocity << " throttle is " << cmd_vel.linear.x << std::endl;
  // std::cout << "lateral error is " << lat_error << " heading_error is " << heading_error << " steering is " << cmd_vel.angular.z << std::endl;

  return cmd_vel;
}

double PathTrackerNode::computeFeedbackKeppa(const double distance, const double heading_error, const double goal_heading)
{
  return (1/distance*(Feedback_K2 * (heading_error - std::atan(-Feedback_K1 * goal_heading)) + ((1 + Feedback_K1/(1 + std::pow(Feedback_K1*goal_heading, 2))) * std::sin(heading_error))));
}

double PathTrackerNode::computeFeedbackSpeed(const double keppa, const double max_vel, const double miu, const double lambda)
{
  return(max_vel/(1+miu*std::pow(std::abs(keppa), lambda)));
}

geometry_msgs::Twist PathTrackerNode::computeFeedbackControl(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal)
{
  // Heading Error
  tf2::Quaternion q_robot, q_goal;
  tf2::fromMsg(odom_robot.pose.pose.orientation, q_robot);
  tf2::fromMsg(pose_goal.orientation, q_goal);
  const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
  const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_goal);

  double roll, pitch, yaw_robot, yaw_goal;
  m_robot.getRPY(roll, pitch, yaw_robot);
  m_goal.getRPY(roll, pitch, yaw_goal);

  const double heading_error = unifyAngleRange(yaw_robot - yaw_goal);

  // Lateral Error
  tf2::Vector3 point_robot, point_goal;
  tf2::fromMsg(odom_robot.pose.pose.position, point_robot);
  tf2::fromMsg(pose_goal.position, point_goal);
  const tf2::Vector3 V_goal_robot = point_robot - point_goal;
  const double angle_goal_robot = std::atan2(V_goal_robot.getY(), V_goal_robot.getX()); // 机器人与目标连线角度
  const double angle_diff = angle_goal_robot - yaw_goal; // 机器人与目标连线与机器人朝向夹角
  const double lat_error = V_goal_robot.length()*std::sin(angle_diff);

  // Distance 
  const double distance = std::sqrt(std::pow(V_goal_robot.getX(), 2) + std::pow(V_goal_robot.getY(), 2));

  // Goal angle
  const double goal_heading = yaw_goal;

  // Velocity
  tf2::Vector3 robot_vel;
  tf2::fromMsg(this->odom_world_robot_.twist.twist.linear, robot_vel);
  const double velocity = robot_vel.length();

  geometry_msgs::Twist cmd_vel;
  double keppa = computeFeedbackKeppa(distance, angle_diff, goal_heading);
  cmd_vel.linear.x = computeFeedbackSpeed(keppa, 1.0, 1, 3);
  cmd_vel.angular.z = cmd_vel.linear.x * keppa;

  return cmd_vel;
}


// Puresuit

MyPathTrackerNode::MyPathTrackerNode() : tf2_listener_(tf2_buffer_)
{
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &MyPathTrackerNode::robotOdomCallback, this);
  this->sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &MyPathTrackerNode::localPathCallback, this);
  this->pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);

  // Initialization
  this->robot_frame_ = "base_link";
  this->world_frame_ = "world";

  this->pid_ = control::PID(0.1, 1.0, -1.0, PID_Kp, PID_Ki, PID_Kd);
}

void MyPathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
{
  // Calculate absolute errors (wrt to world frame)
  this->plan_path = *path;
  this->pose_world_goal_ = path->poses[11].pose;
  // this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_, this->pose_world_goal_));
  this->pub_cmd_vel_.publish(purePursuitOutput(this->odom_world_robot_, this->plan_path));

  return;
}

void MyPathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->odom_world_robot_ = *odom.get();

  return;
}

double MyPathTrackerNode::computeStanelyControl(const double heading_error, const double cross_track_error, const double velocity)
{
  const double stanley_output = -1.0*(heading_error + std::atan2(STANLEY_K*cross_track_error, std::max(velocity, 0.3)));

  return std::min(std::max(stanley_output, -2.2), 2.2);
}

geometry_msgs::Twist MyPathTrackerNode::purePursuitOutput(const nav_msgs::Odometry& odom_robot, const nav_msgs::Path& plan_path)
{
  geometry_msgs::Pose pose_goal;

  // Find the cloest waypoint
  int index_min = 0;
  double min_distance = INT16_MAX;
  for(int i = 0; i < plan_path.poses.size(); i++)
  {
    double distance = std::sqrt(std::pow(plan_path.poses[i].pose.position.x - odom_robot.pose.pose.position.x, 2) + std::pow(plan_path.poses[i].pose.position.y - odom_robot.pose.pose.position.y, 2));
    if(distance < min_distance)
    {
      min_distance = distance;
      index_min = i;
    }
  }

  // Find the 1st waypoint away from LOOKAHEAD_DISTANCE
  int index_lookahead = index_min;
  for(int i = index_min; i < plan_path.poses.size(); i++)
  {
    double distance = std::sqrt(std::pow(plan_path.poses[i].pose.position.x - odom_robot.pose.pose.position.x, 2) + std::pow(plan_path.poses[i].pose.position.y - odom_robot.pose.pose.position.y, 2));
    if(distance > LOOKAHEAD_DISTANCE)
    {
      index_lookahead = i;
      break;
    }
  }

  pose_goal = plan_path.poses[index_lookahead].pose;

  // Transform the 2 waypoints into robot frame

  // Heading Error
  tf2::Quaternion q_robot, q_goal;
  tf2::fromMsg(odom_robot.pose.pose.orientation, q_robot);
  tf2::fromMsg(pose_goal.orientation, q_goal);
  const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
  const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_goal);

  double roll, pitch, yaw_robot, yaw_goal;
  m_robot.getRPY(roll, pitch, yaw_robot);
  m_goal.getRPY(roll, pitch, yaw_goal);

  const double heading_error = unifyAngleRange(yaw_robot - yaw_goal);

  // Lateral Error
  tf2::Vector3 point_robot, point_goal;
  tf2::fromMsg(odom_robot.pose.pose.position, point_robot);
  tf2::fromMsg(pose_goal.position, point_goal);
  const tf2::Vector3 V_goal_robot = point_robot - point_goal;
  const double angle_goal_robot = std::atan2(V_goal_robot.getY(), V_goal_robot.getX());
  const double angle_diff = angle_goal_robot - yaw_goal;
  const double lat_error = V_goal_robot.length()*std::sin(angle_diff);

  // Velocity
  tf2::Vector3 robot_vel;
  tf2::fromMsg(this->odom_world_robot_.twist.twist.linear, robot_vel);
  const double velocity = robot_vel.length();

  geometry_msgs::Twist cmd_vel;
  if (PARAMS_UPDATED)
  {
    this->pid_.updateSettings(PID_Kp, PID_Ki, PID_Kd);
    PARAMS_UPDATED = false;
  }
  cmd_vel.linear.x = this->pid_.calculate(SPEED_TARGET, velocity);
  cmd_vel.angular.z = computeStanelyControl(heading_error, lat_error, velocity);

  // std::cout << "robot velocity is " << velocity << " throttle is " << cmd_vel.linear.x << std::endl;
  // std::cout << "lateral error is " << lat_error << " heading_error is " << heading_error << " steering is " << cmd_vel.angular.z << std::endl;

  return cmd_vel;
}


} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_tracker_node");
  // me5413_world::PathTrackerNode path_tracker_node;
  me5413_world::MyPathTrackerNode path_tracker_node;
  ros::spin();  // spin the ros node.
  return 0;
}