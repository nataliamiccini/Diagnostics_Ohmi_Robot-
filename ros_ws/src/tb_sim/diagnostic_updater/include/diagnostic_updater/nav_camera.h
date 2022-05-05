
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include "ros/ros.h"
#include <iostream>
#include <XmlRpcValue.h>
#include <sensor_msgs/Image.h> //This is a message to hold data from an Image
#ifdef ERROR
#undef ERROR
#endif

using namespace diagnostic_updater;

int nav_camera_length;
int nav_camera_zero;

//Method executed at the call.
void callback3(const sensor_msgs::ImageConstPtr& info_msg){
  nav_camera_length = info_msg->step;
}

/*This method is used to check if the nav camera is on or off. 
  This is done by checking if the message length is null or not.*/
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



