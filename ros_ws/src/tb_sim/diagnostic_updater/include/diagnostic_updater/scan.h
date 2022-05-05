
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include "ros/ros.h"
#include <iostream>
#include <XmlRpcValue.h>
#include <sensor_msgs/LaserScan.h>
#ifdef ERROR
#undef ERROR
#endif


using namespace diagnostic_updater;

float range_min;
float range_max;
std::vector<float> ranges;
float lenght_ranges;
float param_scan;

//Method executed at the call.
void callback4(const sensor_msgs::LaserScan::ConstPtr& msg){
  range_min = msg->range_min;
  range_max = msg->range_max;
  ranges = msg->ranges;
  lenght_ranges = (msg->ranges).size();
}

/* This method is used to check if there is a fault in Scan topic.
   In particular it is done by checking if the ranges are within the range or out of range. 
   Then we check if the scan is on or off by checking the lenght of the ranges. 
   If the lenght is zero, the scan is offline. */

void check_scan(diagnostic_updater::DiagnosticStatusWrapper &stat)
{ 
for (int i = 0; i < lenght_ranges; i++){
  if (ranges[i] > range_min && ranges[i] < range_max)
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Scan OK");
  else if (ranges[i] > range_max)
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Scan ERROR, range Too high [%f]", ranges[i]);
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Scan ERROR, range Too low [%f]", ranges[i]);
}
  stat.add("renge min", range_min);
  stat.add("range max", range_max);
}

void check_scan_on_off(diagnostic_updater::DiagnosticStatusWrapper &stat)
{  
  if (lenght_ranges == param_scan)
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Scan OFF ");
  else 
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Scan ON");
  stat.add("Lenght ranges", lenght_ranges);
}




