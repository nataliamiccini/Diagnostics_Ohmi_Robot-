
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include "ros/ros.h"
#include <iostream>
#include <XmlRpcValue.h>
#ifdef ERROR
#undef ERROR
#endif

using namespace diagnostic_updater;

float ang_vel_x;
float ang_vel_y;
float ang_vel_z;
float param_imu_min;
float param_imu_max;


void callback(const sensor_msgs::Imu::ConstPtr& msg){
  ang_vel_x = msg->angular_velocity.x;
  ang_vel_y = msg->angular_velocity.y;
  ang_vel_z = msg->angular_velocity.z;
}

void check_soglie_x(diagnostic_updater::DiagnosticStatusWrapper &stat)
{  
  if (ang_vel_x > param_imu_min && ang_vel_x < param_imu_max)
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "angular velocity x OK [%f]", ang_vel_x);
  else if ( ang_vel_x > param_imu_max)
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "angular velocity x Too high [%f]", ang_vel_x);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "angular velocity x Too low [%f]", ang_vel_x);
  stat.add("Top-Side Margin x", 10 - ang_vel_x);
  stat.add("imu max", param_imu_max);
  stat.add("imu min", param_imu_min);
}

void check_soglie_y(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (ang_vel_y > param_imu_min && ang_vel_y < param_imu_max)
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "angular velocity y OK [%f]", ang_vel_y);
  else if ( ang_vel_y > param_imu_max)
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "angular velocity y Too high [%f]", ang_vel_y);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "angular velocity y Too low [%f]", ang_vel_y);
  stat.add("Top-Side Margin y", 10 - ang_vel_y);
}

void check_soglie_z(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (ang_vel_z > param_imu_min && ang_vel_z < param_imu_max)
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "angular velocity z OK [%f]", ang_vel_z);
  else if ( ang_vel_z > param_imu_max )
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "angular velocity z Too high[%f]", ang_vel_z);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "angular velocity z Too low[%f]", ang_vel_z);
  stat.add("Top-Side Margin z", 10 - ang_vel_z);
}



