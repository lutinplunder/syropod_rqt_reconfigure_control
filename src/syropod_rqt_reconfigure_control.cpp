////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Oshada Jayasinghe
// Modified: Benjamin Tam
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "syropod_rqt_reconfigure_control/syropod_rqt_reconfigure_control.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


SyropodRqtReconfigureControl::SyropodRqtReconfigureControl()
{
  ros::NodeHandle n;
  desired_velocity_pub_ = n.advertise<geometry_msgs::Twist>("syropod_remote/desired_velocity", 1);
  system_state_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/system_state", 1);
  robot_state_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/robot_state", 1);
  gait_selection_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/gait_selection", 1);
  cruise_control_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/cruise_control_mode", 1);

  dynamic_reconfigure_server_ = new dynamic_reconfigure::Server<syropod_rqt_reconfigure_control::DynamicConfig>();
  dynamic_reconfigure::Server<syropod_rqt_reconfigure_control::DynamicConfig>::CallbackType callback_type;
  callback_type = boost::bind(&SyropodRqtReconfigureControl::dynamicParameterCallback, this, _1, _2);
  dynamic_reconfigure_server_->setCallback(callback_type);
}

void SyropodRqtReconfigureControl::dynamicParameterCallback(syropod_rqt_reconfigure_control::DynamicConfig &config,
                                                            uint32_t level)
{
  if (config.stop == true)
  {
    desired_velocity_msg_.linear.x = 0;
    desired_velocity_msg_.linear.y = 0;
    desired_velocity_msg_.linear.z = 0;
    desired_velocity_msg_.angular.x = 0;
    desired_velocity_msg_.angular.y = 0;
    desired_velocity_msg_.angular.z = 0;
  }
  else if (config.stop == false)
  {
    desired_velocity_msg_.linear.x = config.linear_x;
    desired_velocity_msg_.linear.y = config.linear_y;
    desired_velocity_msg_.linear.z = 0;
    desired_velocity_msg_.angular.x = 0;
    desired_velocity_msg_.angular.y = 0;
    desired_velocity_msg_.angular.z = config.angular_z;
  }
  system_state_msg_.data = config.system_state;
  robot_state_msg_.data = config.robot_state;
  gait_selection_msg_.data = config.gait_selection;
  cruise_control_mode_msg_.data = config.cruise_control_mode;
}

void SyropodRqtReconfigureControl::publishUpdate()
{
  system_state_pub_.publish(system_state_msg_);
  robot_state_pub_.publish(robot_state_msg_);
  desired_velocity_pub_.publish(desired_velocity_msg_);
  gait_selection_pub_.publish(gait_selection_msg_);
  cruise_control_pub_.publish(cruise_control_mode_msg_);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "syropod_rqt_reconfigure_control");

  SyropodRqtReconfigureControl syropod_rqt_reconfigure_control;

  ros::Rate loopRate(LOOP_RATE);
  while (ros::ok())
  {
    syropod_rqt_reconfigure_control.publishUpdate();
    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
