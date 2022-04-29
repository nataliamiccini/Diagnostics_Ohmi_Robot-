
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include "ros/ros.h"
#include <iostream>
#include <XmlRpcValue.h>
#include <diagnostic_updater/scan.h>
#ifdef ERROR
#undef ERROR
#endif


int main(int argc, char **argv)
{
  ros::init(argc, argv, "diagnostic_updater_scan");
  
  ros::NodeHandle nh;
  nh.getParam("param_scan", param_scan);
  diagnostic_updater::Updater updaterScan;

  updaterScan.setHardwareID("/scan");
  ros::Subscriber subScan = nh.subscribe("/scan", 1000, callback4);
  diagnostic_updater::FunctionDiagnosticTask scan("Chek scan",
       boost::bind(&check_scan, boost::placeholders::_1));
  diagnostic_updater::FunctionDiagnosticTask scan_on_off("Chek scan on/off",
       boost::bind(&check_scan_on_off, boost::placeholders::_1));
  updaterScan.add(scan);
  updaterScan.add(scan_on_off);

  updaterScan.broadcast(0, "Doing important initialization stuff.");

  updaterScan.force_update();

  while (nh.ok())
  {
    
    ros::Duration(0.1).sleep();

    ros::spinOnce();

    updaterScan.update();
  }

  return 0; 
}


