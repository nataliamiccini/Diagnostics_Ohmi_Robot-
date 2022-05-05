
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include "ros/ros.h"
#include <iostream>
#include <XmlRpcValue.h>
#include <diagnostic_updater/tele_camera.h>
#include <diagnostic_updater/nav_camera.h>
#ifdef ERROR
#undef ERROR
#endif


int main(int argc, char **argv)
{
  //Ros initialization function.
  ros::init(argc, argv, "diagnostic_updater_nav_cam");
  //Construct a NodeHandle Class. This class is used for writing nodes.
  ros::NodeHandle nh;
  //Get parameters from .yaml file and store them into indicated variables.
  nh.getParam("nav_camera_zero", nav_camera_zero);
  
  //Construct an updater class.
  diagnostic_updater::Updater updater4;

  updater4.setHardwareID("/nav_camera/image_raw");
  
  /*Manages a subscription callback on /nav_camera/image_raw topic 
  and assigns a maximum size to the queue*/
  ros::Subscriber sub3 = nh.subscribe("/nav_camera/image_raw", 1000, callback3);
  //Construct a Diagnostic Task 
  diagnostic_updater::FunctionDiagnosticTask nav_camera("Chek nav camera",
       boost::bind(&check_nav_camera, boost::placeholders::_1));
  //Add the DiagnosticTask to our Updater.
  updater4.add(nav_camera);

  updater4.broadcast(0, "Doing important initialization stuff.");
  
  updater4.force_update();
 
  while (nh.ok())
  {
    
    ros::Duration(0.1).sleep();
    
    /*spinOnce() will call all the 
    callbacks waiting to be called at that point in time.*/
    ros::spinOnce();
    
    //Call updater
    updater4.update();

  }

  return 0; 
}


