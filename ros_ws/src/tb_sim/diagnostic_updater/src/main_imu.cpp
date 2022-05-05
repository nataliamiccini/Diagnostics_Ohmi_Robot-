
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include "ros/ros.h"
#include <iostream>
#include <XmlRpcValue.h>
#include <diagnostic_updater/imu.h>
#ifdef ERROR
#undef ERROR
#endif


int main(int argc, char **argv)
{
  ros::init(argc, argv, "diagnostic_updater_imu") //Ros initialization function.
  ros::NodeHandle nh; //Construct a NodeHandle Class. This class is used for writing nodes. 
  
  nh.getParam("param_imu_min", param_imu_min); //Get parameters from .yaml file and store them into indicated variables.
  nh.getParam("param_imu_max", param_imu_max);
  
  diagnostic_updater::Updater updaterImu; //Construct an updater class.
  
  updaterImu.setHardwareID("/tb_sim/imu");

  /*Manages a subscription callback on /tb_sim/imu topic 
  and assigns a maximum size to the queue*/
  ros::Subscriber subImu = nh.subscribe("/tb_sim/imu", 1000, callback);
  /*Construct a Diagnostic Task and combine multiple diagnostic
  tasks using a CompositeDiagnosticTask.*/
  diagnostic_updater::FunctionDiagnosticTask sogliex("Chek soglie x",
       boost::bind(&check_soglie_x, boost::placeholders::_1));
  diagnostic_updater::FunctionDiagnosticTask sogliey("Chek soglie y",
       boost::bind(&check_soglie_y, boost::placeholders::_1));
  diagnostic_updater::FunctionDiagnosticTask sogliez("Chek soglie z",
       boost::bind(&check_soglie_z, boost::placeholders::_1));
  diagnostic_updater::CompositeDiagnosticTask bounds("Bound check");
  //Creates a new task, registers the task, and returns the instance.
  bounds.addTask(&sogliex);
  bounds.addTask(&sogliey);
  bounds.addTask(&sogliez);
  //Add the CompositeDiagnosticTask to our Updater.
  updaterImu.add(bounds);


  updaterImu.broadcast(0, "Doing important initialization stuff.");
 
  updaterImu.force_update();
 

  while (nh.ok())
  {
    
    ros::Duration(0.1).sleep();
    //spinOnce() will call all the callbacks waiting to be called at that point in time.
    ros::spinOnce();
    //Call updater
    updaterImu.update();
    
  }

  return 0; 
}


