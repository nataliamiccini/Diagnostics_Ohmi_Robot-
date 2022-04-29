
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include "ros/ros.h"
#include <iostream>
#include <sensor_msgs/JointState.h>
#include "robot_state_publisher/robot_state_publisher.h"
#include "robot_state_publisher/joint_state_listener.h"
#include <kdl_parser/kdl_parser.hpp>
#include <vector>
#include <string>
#ifdef ERROR
#undef ERROR
#endif
using namespace std;
using namespace diagnostic_updater;

double wheel_left_joint;
double wheel_right_joint;
double caster_joint;
double caster_wheel_joint;
double neck_joint;
double param_joint;


void callback2(const sensor_msgs::JointState::ConstPtr& msg){
   wheel_left_joint = msg->position[0];
   wheel_right_joint = msg->position[1];
   caster_joint = msg->position[2];
   caster_wheel_joint = msg->position[3];
   neck_joint = msg->position[4];
}

void check_wheel_left_joint(diagnostic_updater::DiagnosticStatusWrapper &stat)
{ 
  if ( wheel_left_joint < param_joint ){
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Invalid wheel left joint [%f]", wheel_left_joint);
  }
  else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Position of wheel left joint OK");
  }
  stat.add("wheel left joint", wheel_left_joint);
}

void check_wheel_right_joint(diagnostic_updater::DiagnosticStatusWrapper &stat)
{ 
  if ( wheel_right_joint < param_joint ){
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Invalid wheel right joint [%f]", wheel_right_joint);
  }
  else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Position of wheel right joint OK");
  }
  stat.add("wheel right joint", wheel_right_joint);
}

void check_caster_joint(diagnostic_updater::DiagnosticStatusWrapper &stat)
{ 

  if ( caster_joint < param_joint ){
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Invalid caster joint [%f]", caster_joint);
  }
  else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Position of caster joint OK");
  }
  stat.add("caster joint", caster_joint);
}

//ruota dietro
void check_caster_wheel_joint(diagnostic_updater::DiagnosticStatusWrapper &stat)
{ 

  if ( caster_wheel_joint < param_joint ){
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Invalid caster wheel joint [%f]", caster_wheel_joint);
  }
  else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Position of caster wheel joint OK");
  }
  stat.add("caster wheel joint", caster_wheel_joint);
}

void check_neck_joint(diagnostic_updater::DiagnosticStatusWrapper &stat)
{ 

  if ( neck_joint < param_joint ){
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Invalid neck joint [%f]", neck_joint);
  }
  else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Position of neck joint OK");
  }
  stat.add("neck joint", neck_joint);
}




