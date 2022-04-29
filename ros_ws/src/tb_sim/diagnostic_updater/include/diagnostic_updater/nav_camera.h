
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include "ros/ros.h"
#include <iostream>
#include <XmlRpcValue.h>
#include <sensor_msgs/Image.h>
#ifdef ERROR
#undef ERROR
#endif

using namespace diagnostic_updater;

int nav_camera_length;
int nav_camera_zero;

void callback3(const sensor_msgs::ImageConstPtr& info_msg){
  nav_camera_length = info_msg->step;
}

void check_nav_camera(diagnostic_updater::DiagnosticStatusWrapper &stat)
{  
  if (nav_camera_length == nav_camera_zero){
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Nav Camera OFF [%i]", nav_camera_length);
  }
  else{ 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Nav Camera OK [%i]", nav_camera_length);
  }
  stat.add("nav_camera", nav_camera_length);
}



