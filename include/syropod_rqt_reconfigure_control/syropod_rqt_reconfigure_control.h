////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Oshada Jayasinghe
// Modified: Benjamin Tam
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SYROPOD_RQT_RECONFIGURE_CONTROL_H
#define SYROPOD_RQT_RECONFIGURE_CONTROL_H

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <syropod_highlevel_controller/parameters_and_states.h>
#include <syropod_highlevel_controller/standard_includes.h>
#include <syropod_rqt_reconfigure_control/DynamicConfig.h>

#define LOOP_RATE 10

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class SyropodRqtReconfigureControl
{
public:
  /// Contructor for syropod_rqt_reconfigure_control object.
  SyropodRqtReconfigureControl();
  /// Callback handling the dynamic reconfigure control of syropod
  void dynamicParameterCallback(syropod_rqt_reconfigure_control::DynamicConfig &config, uint32_t level);
  /// Publish the updated messages
  void publishUpdate();

private:
  ros::Publisher desired_velocity_pub_;  ///< Publisher for topic "syropod_remote/desired_velocity"
  ros::Publisher system_state_pub_;      ///< Publisher for topic "syropod_remote/system_state"
  ros::Publisher robot_state_pub_;       ///< Publisher for topic "syropod_remote/robot_state"
  ros::Publisher gait_selection_pub_;    ///< Publisher for topic "syropod_remote/gait_selection"
  ros::Publisher cruise_control_pub_;    ///< Publisher for topic "syropod_remote/cruise_control_mode"

  /// Dynamic reconfigure server pointer
  dynamic_reconfigure::Server<syropod_rqt_reconfigure_control::DynamicConfig> *dynamic_reconfigure_server_;

  geometry_msgs::Twist desired_velocity_msg_;  ///< Message published on "syropod_remote/desired_velocity"
  std_msgs::Int8 system_state_msg_;            ///< Message published on "syropod_remote/system_state"
  std_msgs::Int8 robot_state_msg_;             ///< Message published on "syropod_remote/robot_state"
  std_msgs::Int8 gait_selection_msg_;          ///< Message published on "syropod_remote/gait_selection"
  std_msgs::Int8 cruise_control_mode_msg_;     ///< Message published on "syropod_remote/cruise_control_mode"
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif  // SYROPOD_RQT_RECONFIGURE_CONTROL_H