
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
  ros::init(argc, argv, "diagnostic_updater_nav_cam");
  
  ros::NodeHandle nh;
  nh.getParam("nav_camera_zero", nav_camera_zero);

  diagnostic_updater::Updater updater4;

  updater4.setHardwareID("/nav_camera/image_raw");
  ros::Subscriber sub3 = nh.subscribe("/nav_camera/image_raw", 1000, callback3);
  diagnostic_updater::FunctionDiagnosticTask nav_camera("Chek nav camera",
       boost::bind(&check_nav_camera, boost::placeholders::_1));
  updater4.add(nav_camera);


  updater4.broadcast(0, "Doing important initialization stuff.");
  
  updater4.force_update();
 
  while (nh.ok())
  {
    
    ros::Duration(0.1).sleep();

    ros::spinOnce();

    updater4.update();

  }

  return 0; 
}


