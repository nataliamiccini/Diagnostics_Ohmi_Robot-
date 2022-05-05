
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include "ros/ros.h"
#include <iostream>
#include <XmlRpcValue.h>
#include <diagnostic_updater/joint_states.h>
#ifdef ERROR
#undef ERROR
#endif


int main(int argc, char **argv)
{
  //Ros initialization function.
  ros::init(argc, argv, "diagnostic_updater_joint_states");
  //Construct a NodeHandle Class. This class is used for writing nodes.
  ros::NodeHandle nh;
  //Get parameters from .yaml file and store them into indicated variables.
  nh.getParam("param_joint", param_joint);
  
  //Construct an updater class.
  diagnostic_updater::Updater updaterJoint;

  updaterJoint.setHardwareID("/joint_states");
  /*Manages a subscription callback on /joint_states topic 
  and assigns a maximum size to the queue*/
  ros::Subscriber sub2 = nh.subscribe("/joint_states", 1000, callback2);
  diagnostic_updater::FunctionDiagnosticTask joint1("Chek joint 1",
       boost::bind(&check_wheel_left_joint, boost::placeholders::_1));
  diagnostic_updater::FunctionDiagnosticTask joint2("Chek joint 2",
       boost::bind(&check_wheel_right_joint, boost::placeholders::_1));
  diagnostic_updater::FunctionDiagnosticTask joint3("Chek joint 3",
       boost::bind(&check_caster_joint, boost::placeholders::_1));
  diagnostic_updater::FunctionDiagnosticTask joint4("Chek joint 4",
       boost::bind(&check_caster_wheel_joint, boost::placeholders::_1));
  diagnostic_updater::FunctionDiagnosticTask joint5("Chek joint 5",
       boost::bind(&check_neck_joint, boost::placeholders::_1));
  diagnostic_updater::CompositeDiagnosticTask Joints("Bound check Joints");
  //Creates a new task, registers the task, and returns the instance.
  Joints.addTask(&joint1);
  Joints.addTask(&joint2);
  Joints.addTask(&joint3);
  Joints.addTask(&joint4);
  Joints.addTask(&joint5);
  //Add the CompositeDiagnosticTask to our Updater.
  updaterJoint.add(Joints);


  updaterJoint.broadcast(0, "Doing important initialization stuff.");

  updaterJoint.force_update();
  

  while (nh.ok())
  {
    
    ros::Duration(0.1).sleep();
    
    /*spinOnce() will call all the 
    callbacks waiting to be called at that point in time.*/ 
    ros::spinOnce();
    
    //Call updater
    updaterJoint.update();

  }

  return 0; 
}


