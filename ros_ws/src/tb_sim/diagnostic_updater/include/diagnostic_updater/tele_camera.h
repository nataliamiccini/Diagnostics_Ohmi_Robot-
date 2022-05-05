
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

//Method executed at the call.
void callback1(const sensor_msgs::ImageConstPtr& info_msg){
  tele_camera_length = info_msg->step;
}


/* This method is used to check if there is a fault in Tele Camera topic.
   In particular it is done by checking if the lenght of the camera is zero.
   If the lenght of the camera is zero then the tele camera is offline.*/
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


/*
void cameraCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
 {
     camera_h = info_msg->height;
     camera_w = info_msg->width;
 }

void check_camera(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
 if(camera_h!=camera_off && camera_w!=camera_off)
 stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Camera on: [%i] [%i]", camera_w, camera_h);
 else
 stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Camera off: [%i] [%i]", camera_w, camera_h);
}
*/
