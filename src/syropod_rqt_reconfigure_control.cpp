////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Fletcher Talbot
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <syropod_highlevel_controller/standard_includes.h>
#include <syropod_highlevel_controller/parameters_and_states.h>

#include <dynamic_reconfigure/server.h>
#include <syropod_rqt_reconfigure_control/DynamicConfig.h>

geometry_msgs::Twist desired_velocity_msg_;
std_msgs::Int8 system_state_msg_;
std_msgs::Int8 robot_state_msg_;
std_msgs::Int8 gait_selection_msg_;
std_msgs::Int8 cruise_control_mode_msg_;
std_msgs::Int8 posing_mode_msg_;

void dynamicParameterCallback(syropod_rqt_reconfigure_control::DynamicConfig &config, uint32_t level)
{
  desired_velocity_msg_.linear.x = config.linear_x;
  desired_velocity_msg_.linear.y = config.linear_y;
  desired_velocity_msg_.linear.z = 0; 
  desired_velocity_msg_.angular.x = 0;
  desired_velocity_msg_.angular.y = 0;
  desired_velocity_msg_.angular.z = config.angular_z;
  system_state_msg_.data = config.system_state;
  robot_state_msg_.data = config.robot_state;
  gait_selection_msg_.data = config.gait_selection;
  cruise_control_mode_msg_.data = config.cruise_control_mode;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "syropod_rqt_reconfigure_control");

  ros::NodeHandle n;

  ros::Publisher desired_velocity_pub_ = n.advertise<geometry_msgs::Twist>("syropod_remote/desired_velocity",1);
  ros::Publisher system_state_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/system_state", 1);
  ros::Publisher robot_state_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/robot_state", 1);
  ros::Publisher gait_selection_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/gait_selection", 1);
  ros::Publisher cruise_control_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/cruise_control_mode", 1);
  
  ros::Rate loopRate(10);

  dynamic_reconfigure::Server<syropod_rqt_reconfigure_control::DynamicConfig> server;
  dynamic_reconfigure::Server<syropod_rqt_reconfigure_control::DynamicConfig>::CallbackType callback_type;
  callback_type = boost::bind(&dynamicParameterCallback, _1, _2);
  server.setCallback(callback_type);
  
  while(ros::ok()){

  system_state_pub_.publish(system_state_msg_);
  robot_state_pub_.publish(robot_state_msg_);
  desired_velocity_pub_.publish(desired_velocity_msg_);
  gait_selection_pub_.publish(gait_selection_msg_);
  cruise_control_pub_.publish(cruise_control_mode_msg_);	

  ros::spinOnce();
  loopRate.sleep();
  }
  
  return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
