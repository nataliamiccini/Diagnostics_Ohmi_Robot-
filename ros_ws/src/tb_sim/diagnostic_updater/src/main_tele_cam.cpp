
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
  ros::init(argc, argv, "diagnostic_updater_tele_cam");
  //Construct a NodeHandle Class. This class is used for writing nodes. 
  ros::NodeHandle nh;
  //Get parameters from .yaml file and store them into indicated variables.
  nh.getParam("tele_camera_zero", tele_camera_zero);

  diagnostic_updater::Updater updaterTele; //Construct an updater class.

  updaterTele.setHardwareID("/tele_camera/image_raw");
  /*Manages a subscription callback on /tele_camera/image_raw topic 
  and assigns a maximum size to the queue*/
  ros::Subscriber subTele = nh.subscribe("/tele_camera/image_raw", 1000, callback1);
  //Construct a Diagnostic Task 
  diagnostic_updater::FunctionDiagnosticTask tele_camera("Chek tele camera",
       boost::bind(&check_tele_camera, boost::placeholders::_1));
  //Add the DiagnosticTask to our Updater.
  updaterTele.add(tele_camera);


  updaterTele.broadcast(0, "Doing important initialization stuff.");
  
  updaterTele.force_update();
 
  while (nh.ok())
  {
    
    ros::Duration(0.1).sleep();
    
    /*spinOnce() will call all the 
    callbacks waiting to be called at that point in time.*/
    ros::spinOnce();
    
    //Call updater
    updaterTele.update();
    
  }

  return 0; 
}


