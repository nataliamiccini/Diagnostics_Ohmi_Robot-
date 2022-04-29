
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
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

int tele_camera_length;
int tele_camera_zero;

void callback1(const sensor_msgs::ImageConstPtr& info_msg){
  tele_camera_length = info_msg->step;
}

void check_tele_camera(diagnostic_updater::DiagnosticStatusWrapper &stat)
{  
  if (tele_camera_length == tele_camera_zero){
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Tele Camera OFF [%i]", tele_camera_length);
  }
  else{ 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Tele Camera OK [%i]", tele_camera_length);
  }
  stat.add("tele_camera", tele_camera_length);
}



