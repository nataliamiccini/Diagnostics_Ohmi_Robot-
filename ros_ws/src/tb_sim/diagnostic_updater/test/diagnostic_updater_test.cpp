/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/**#include <gtest/gtest.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#ifndef _WIN32
#include <unistd.h>
#endif

using namespace diagnostic_updater;

class TestClass 
{
public: 
  void unwrapped(diagnostic_msgs::DiagnosticStatus &s)
  {
  }

  void wrapped(DiagnosticStatusWrapper &s)
  {
  }
};
                                   
TEST(DiagnosticUpdater, testDiagnosticUpdater)
{
  class classFunction : public DiagnosticTask
  {
  public:
    classFunction() : DiagnosticTask("classFunction")
    {}

    void run(DiagnosticStatusWrapper &s) 
    {
      s.summary(0, "Test is running");
      s.addf("Value", "%f", 5);
      s.add("String", "Toto");
      s.add("Floating", 5.55);
      s.add("Integer", 5);
      s.addf("Formatted %s %i", "Hello", 5);
      s.add("Bool", true);
    }
  };
  
  TestClass c;
  ros::NodeHandle nh;
  
  Updater updater;
  
  updater.add("wrapped", &c, &TestClass::wrapped);
  
  classFunction cf;
  updater.add(cf);
}

TEST(DiagnosticUpdater, testDiagnosticStatusWrapperKeyValuePairs)
{
  DiagnosticStatusWrapper stat;
  
  const char *message = "dummy";
  int level = 1;
  stat.summary(level, message);
  EXPECT_STREQ(message, stat.message.c_str()) << "DiagnosticStatusWrapper::summary failed to set message";
  EXPECT_EQ(level, stat.level) << "DiagnosticStatusWrapper::summary failed to set level";

  stat.addf("toto", "%.1f", 5.0);
  stat.add("baba", 5);
  stat.addf("foo", "%05i", 27);

  stat.add("bool", true);
  stat.add("bool2", false);
  
  EXPECT_STREQ("5.0", stat.values[0].value.c_str()) << "Bad value, adding a value with addf";
  EXPECT_STREQ("5", stat.values[1].value.c_str()) << "Bad value, adding a string with add";
  EXPECT_STREQ("00027", stat.values[2].value.c_str()) << "Bad value, adding a string with addf";
  EXPECT_STREQ("toto", stat.values[0].key.c_str()) << "Bad label, adding a value with add";
  EXPECT_STREQ("baba", stat.values[1].key.c_str()) << "Bad label, adding a string with add";
  EXPECT_STREQ("foo", stat.values[2].key.c_str()) << "Bad label, adding a string with addf";

  EXPECT_STREQ("bool", stat.values[3].key.c_str()) << "Bad label, adding a true bool key with add";
  EXPECT_STREQ("True", stat.values[3].value.c_str()) << "Bad label, adding a true bool with add";

  EXPECT_STREQ("bool2", stat.values[4].key.c_str()) << "Bad label, adding a false bool key with add";
  EXPECT_STREQ("False", stat.values[4].value.c_str()) << "Bad label, adding a false bool with add";
}

TEST(DiagnosticUpdater, testDiagnosticStatusWrapperMergeSummary)
{
  DiagnosticStatusWrapper stat;

  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Old");
  stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "New");
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, stat.level) << "Bad level, merging levels (OK,OK)";
  EXPECT_STREQ("Old", stat.message.c_str()) << "Bad summary, merging levels (OK,OK)";

  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Old");
  stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "New");
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, stat.level) << "Bad level, merging levels (OK,WARN)";
  EXPECT_STREQ("New", stat.message.c_str()) << "Bad summary, merging levels (OK,WARN)";

  stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Old");
  stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "New");
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, stat.level) << "Bad level, merging levels (WARN,WARN)";
  EXPECT_STREQ("Old; New", stat.message.c_str()) << "Bad summary, merging levels (WARN,WARN)";

  stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Old");
  stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "New");
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, stat.level) << "Bad level, merging levels (WARN,ERROR)";
  EXPECT_STREQ("Old; New", stat.message.c_str()) << "Bad summary, merging levels (WARN,ERROR)";
}

TEST(DiagnosticUpdater, testFrequencyStatus)
{
  double minFreq = 10;
  double maxFreq = 20;
  
  ros::Time::init();
  ros::Time time(0, 0);
  ros::Time::setNow(time);

  FrequencyStatus fs(FrequencyStatusParam(&minFreq, &maxFreq, 0.5, 2));

  const int MS_TO_NS = 1000000;

  DiagnosticStatusWrapper stat[5];
  fs.tick();
  time += ros::Duration(0, 20 * MS_TO_NS); ros::Time::setNow(time);
  fs.run(stat[0]); // Should be too fast, 20 ms for 1 tick, lower limit should be 33ms.
  time += ros::Duration(0, 50 * MS_TO_NS); ros::Time::setNow(time);
  fs.tick();
  fs.run(stat[1]); // Should be good, 70 ms for 2 ticks, lower limit should be 66 ms.
  time += ros::Duration(0, 300 * MS_TO_NS); ros::Time::setNow(time);
  fs.tick();
  fs.run(stat[2]); // Should be good, 350 ms for 2 ticks, upper limit should be 400 ms.
  time += ros::Duration(0, 150 * MS_TO_NS); ros::Time::setNow(time);
  fs.tick();
  fs.run(stat[3]); // Should be too slow, 450 ms for 2 ticks, upper limit should be 400 ms.
  fs.clear();
  fs.run(stat[4]); // Should be good, just cleared it.

  using diagnostic_msgs::DiagnosticStatus;

  EXPECT_EQ(DiagnosticStatus::WARN, stat[0].level) << "max frequency exceeded but not reported";
  EXPECT_EQ(DiagnosticStatus::OK, stat[1].level) << "within max frequency but reported error";
  EXPECT_EQ(DiagnosticStatus::OK, stat[2].level) << "within min frequency but reported error";
  EXPECT_EQ(DiagnosticStatus::WARN, stat[3].level) << "min frequency exceeded but not reported";
  EXPECT_EQ(DiagnosticStatus::ERROR, stat[4].level) << "freshly cleared should fail";
  EXPECT_STREQ("", stat[0].name.c_str()) << "Name should not be set by FrequencyStatus";
  EXPECT_STREQ("Frequency Status", fs.getName().c_str()) << "Name should be \"Frequency Status\"";
}

TEST(DiagnosticUpdater, testSlowFrequencyStatus)
{
  // We have a slow topic (~0.5 Hz) and call the run() method once a second. This ensures that if the window size
  // is large enough (longer than 1/min_frequency * duration_between_run_calls), the diagnostics correctly reports
  // the frequency status even in time windows where no ticks happened.

  double minFreq = 0.25;
  double maxFreq = 0.75;

  ros::Time::init();
  ros::Time time(0, 0);
  ros::Time::setNow(time);

  FrequencyStatus fs(FrequencyStatusParam(&minFreq, &maxFreq, 0.0, 5));

  DiagnosticStatusWrapper stat[8];
  fs.tick();
  time += ros::Duration(1, 0); ros::Time::setNow(time);
  fs.run(stat[0]); // too high, 1 event in 1 sec window
  time += ros::Duration(1, 0); ros::Time::setNow(time);
  fs.run(stat[1]); // ok, 1 event in 2 sec window
  fs.tick();
  time += ros::Duration(1, 0); ros::Time::setNow(time);
  fs.run(stat[2]); // ok, 2 events in 3 sec window
  time += ros::Duration(1, 0); ros::Time::setNow(time);
  fs.run(stat[3]); // ok, 2 events in 4 sec window
  time += ros::Duration(1, 0); ros::Time::setNow(time);
  fs.run(stat[4]); // ok, 2 events in 5 sec window
  time += ros::Duration(1, 0); ros::Time::setNow(time);
  fs.run(stat[5]); // too low, 1 event in 5 sec window (first tick went out of window)
  time += ros::Duration(1, 0); ros::Time::setNow(time);
  fs.run(stat[6]); // too low, 1 event in 5 sec window (first tick went out of window)
  time += ros::Duration(1, 0); ros::Time::setNow(time);
  fs.run(stat[7]); // no events (second tick went out of window)
  time += ros::Duration(1, 0); ros::Time::setNow(time);

  using diagnostic_msgs::DiagnosticStatus;

  EXPECT_EQ(DiagnosticStatus::WARN, stat[0].level) << "max frequency exceeded but not reported";
  EXPECT_EQ(DiagnosticStatus::OK, stat[1].level) << "within frequency limits but reported error";
  EXPECT_EQ(DiagnosticStatus::OK, stat[2].level) << "within frequency limits but reported error";
  EXPECT_EQ(DiagnosticStatus::OK, stat[3].level) << "within frequency limits but reported error";
  EXPECT_EQ(DiagnosticStatus::OK, stat[4].level) << "within frequency limits but reported error";
  EXPECT_EQ(DiagnosticStatus::WARN, stat[5].level) << "min frequency exceeded but not reported";
  EXPECT_EQ(DiagnosticStatus::WARN, stat[6].level) << "min frequency exceeded but not reported";
  EXPECT_EQ(DiagnosticStatus::ERROR, stat[7].level) << "no events should fail";
}

TEST(DiagnosticUpdater, testTimeStampStatus)
{
  ros::Time::init();
  ros::Time time(1, 0);
  ros::Time::setNow(time);

  TimeStampStatus ts(DefaultTimeStampStatusParam);

  DiagnosticStatusWrapper stat[5];
  ts.run(stat[0]);
  ts.tick(time.toSec() + 2);
  ts.run(stat[1]);
  ts.tick(time);
  ts.run(stat[2]);
  ts.tick(time.toSec() - 4);
  ts.run(stat[3]);
  ts.tick(time.toSec() - 6);
  ts.run(stat[4]);

  using diagnostic_msgs::DiagnosticStatus;

  EXPECT_EQ(DiagnosticStatus::WARN, stat[0].level) << "no data should return a warning";
  EXPECT_EQ(DiagnosticStatus::ERROR, stat[1].level) << "too far future not reported";
  EXPECT_EQ(DiagnosticStatus::OK, stat[2].level) << "now not accepted";
  EXPECT_EQ(DiagnosticStatus::OK, stat[3].level) << "4 seconds ago not accepted";
  EXPECT_EQ(DiagnosticStatus::ERROR, stat[4].level) << "too far past not reported";
  EXPECT_STREQ("", stat[0].name.c_str()) << "Name should not be set by TimeStapmStatus";
  EXPECT_STREQ("Timestamp Status", ts.getName().c_str()) << "Name should be \"Timestamp Status\"";
}

TEST(DiagnosticUpdater, testSlowTimeStampStatus)
{
  // We have a slow topic (< 1 Hz) and call the run() method once a second. If we set the no_data_is_problem parameter
  // to false, updates without data should not generate a warning but should be treated as ok.

  ros::Time::init();
  ros::Time time(1, 0);
  ros::Time::setNow(time);

  SlowTimeStampStatus ts(TimeStampStatusParam(-1, 5));

  DiagnosticStatusWrapper stat[11];
  ts.run(stat[0]); // no events
  ts.tick(time.toSec() + 2);
  ts.run(stat[1]);
  ts.run(stat[2]);
  ts.tick(time.toSec() - 4);
  ts.run(stat[3]);
  ts.run(stat[4]);
  ts.run(stat[5]);
  ts.run(stat[6]);
  ts.tick(time.toSec() - 6);
  ts.run(stat[7]);
  ts.run(stat[8]);
  ts.run(stat[9]);
  ts.run(stat[10]);

  using diagnostic_msgs::DiagnosticStatus;

  EXPECT_EQ(DiagnosticStatus::OK, stat[0].level) << "no data should be ok";
  EXPECT_EQ(DiagnosticStatus::ERROR, stat[1].level) << "too far future not reported";
  EXPECT_EQ(DiagnosticStatus::OK, stat[2].level) << "no data should be ok";
  EXPECT_EQ(DiagnosticStatus::OK, stat[3].level) << "4 seconds ago not accepted";
  EXPECT_EQ(DiagnosticStatus::OK, stat[4].level) << "no data should be ok";
  EXPECT_EQ(DiagnosticStatus::OK, stat[5].level) << "no data should be ok";
  EXPECT_EQ(DiagnosticStatus::OK, stat[6].level) << "no data should be ok";
  EXPECT_EQ(DiagnosticStatus::ERROR, stat[7].level) << "too far past not reported";
  EXPECT_EQ(DiagnosticStatus::OK, stat[8].level) << "no data should be ok";
  EXPECT_EQ(DiagnosticStatus::OK, stat[9].level) << "no data should be ok";
  EXPECT_EQ(DiagnosticStatus::OK, stat[10].level) << "no data should be ok";
}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_node");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}**/

/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>

// ERROR defined in windows.h causes name collision, undefine the macro to fix the issue
#ifdef ERROR
#undef ERROR
#endif

double time_to_launch;

/*
 *\brief Used as a tutorial for loading and using diagnostic updater
 *
 * DummyClass and dummy_diagnostics show how to use a diagnostic_updater
 * class.
 */

void dummy_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  // DiagnosticStatusWrapper are a derived class of 
  // diagnostic_msgs::DiagnosticStatus provides a set of convenience
  // methods.
  
  // summary and summaryf set the level and message.
  if (time_to_launch < 10)
    // summaryf for formatted text.
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Buckle your seat belt. Launch in %f seconds!", time_to_launch);
  else
    // summary for unformatted text.
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Launch is in a long time. Have a soda.");

  // add and addf are used to append key-value pairs.
  stat.add("Diagnostic Name", "dummy");
  // add transparently handles conversion to string (using a string_stream).
  stat.add("Time to Launch", time_to_launch);
  // addf allows arbitrary printf style formatting.
  stat.addf("Geeky thing to say", "The square of the time to launch %f is %f", 
      time_to_launch, time_to_launch * time_to_launch);
}

/*class DummyClass
 {
 public:
   void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
   {
     stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "This is a silly updater.");
 
     stat.add("Stupidicity of this updater", 1000.);
   }
 };

 class DummyTask : public diagnostic_updater::DiagnosticTask
 {
 public:
   DummyTask() : DiagnosticTask("Updater Derived from DiagnosticTask")
   {}
 
   void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
   {
     stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "This is another silly updater.");
     stat.add("Stupidicity of this updater", 2000.);
   }
 }; */

void check_lower_bound(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (time_to_launch > 5)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Lower-bound OK");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Too low");

  stat.add("Low-Side Margin", time_to_launch - 5);
}

void check_upper_bound(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (time_to_launch < 10)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Upper-bound OK");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Too high");

  stat.add("Top-Side Margin", 10 - time_to_launch);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "diagnostic_updater_example");
  
  ros::NodeHandle nh;
  
  // The Updater class advertises to /diagnostics, and has a
  // ~diagnostic_period parameter that says how often the diagnostics
  // should be published.
  diagnostic_updater::Updater updater;

  // The diagnostic_updater::Updater class will fill out the hardware_id
  // field of the diagnostic_msgs::DiagnosticStatus message. You need to
  // use the setHardwareID() or setHardwareIDf() methods to set the
  // hardware ID. 
  //
  // The hardware ID should be able to identify the specific device you are
  // working with.  If it is not appropriate to fill out a hardware ID in
  // your case, you should call setHardwareIDf("none") to avoid warnings.
  // (A warning will be generated as soon as your node updates with no
  // non-OK statuses.)

  //updater.setHardwareIDf("imu", 27, 46);
  updater.setHardwareIDf("Imu");

  // Diagnostic tasks are added to the Updater. They will later be run when
  // the updater decides to update. The add method is heavily overloaded
  // for convenience. Check doxygen for the full list of add methods.

  /*updater.add("Function updater", dummy_diagnostic);
  DummyClass dc;
  updater.add("Method updater", &dc, &DummyClass::produce_diagnostics);*/
  
  // Internally, updater.add converts its arguments into a DiagnosticTask.
  // Sometimes it can be useful to work directly with DiagnosticTasks. Look
  // at FrequencyStatus and TimestampStatus in update_functions.h for a
  // real-life example of how to make a DiagnosticTask by deriving from
  // DiagnosticTask.

  diagnostic_updater::FunctionDiagnosticTask lower("Lower-bound check",
       boost::bind(&check_lower_bound, boost::placeholders::_1));
   diagnostic_updater::FunctionDiagnosticTask upper("Upper-bound check",
       boost::bind(&check_upper_bound, boost::placeholders::_1));

  diagnostic_updater::CompositeDiagnosticTask bounds("Bound check");
   bounds.addTask(&lower);
   bounds.addTask(&upper);

  updater.add(bounds);

  // You can broadcast a message in all the DiagnosticStatus if your node
  // is in a special state.
  updater.broadcast(0, "Doing important initialization stuff.");

  ros::Publisher pub1 = nh.advertise<std_msgs::Imu>("imu", 1);
  ros::Publisher pub2_temp = nh.advertise<std_msgs::Bool>("topic2", 1);
  ros::Duration(2).sleep(); // It isn't important if it doesn't take time.

  // Some diagnostic tasks are very common, such as checking the rate
  // at which a topic is publishing, or checking that timestamps are
  // sufficiently recent. FrequencyStatus and TimestampStatus can do these
  // checks for you. 
  //
  // Usually you would instantiate them via a HeaderlessTopicDiagnostic
  // (FrequencyStatus only, for topics that do not contain a header) or a
  // TopicDiagnostic (FrequencyStatus and TimestampStatus, for topics that
  // do contain a header). 
  //
  // Some values are passed to the constructor as pointers. If these values
  // are changed, the FrequencyStatus/TimestampStatus will start operating
  // with the new values. 
  //
  // Refer to diagnostic_updater::FrequencyStatusParam and
  // diagnostic_updater::TimestampStatusParam documentation for details on
  // what the parameters mean:
  double min_freq = 0.5; // If you update these values, the
  double max_freq = 2; // HeaderlessTopicDiagnostic will use the new values.
  diagnostic_updater::HeaderlessTopicDiagnostic pub1_freq("topic1", updater,
      diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10));

  // Note that TopicDiagnostic, HeaderlessDiagnosedPublisher,
  // HeaderlessDiagnosedPublisher and DiagnosedPublisher all descend from
  // CompositeDiagnosticTask, so you can add your own fields to them using
  // the addTask method.
  //
  // Each time pub1_freq is updated, lower will also get updated and its
  // output will be merged with the output from pub1_freq.
  pub1_freq.addTask(&lower); // (This wouldn't work if lower was stateful).

  // If we know that the state of the node just changed, we can force an
  // immediate update.
  updater.force_update();

  // We can remove a task by refering to its name.
 /* if (!updater.removeByName("Bound check"))
    ROS_ERROR("The Bound check task was not found when trying to remove it.");*/

  while (nh.ok())
  {
    std_msgs::Imu msg;
    ros::Duration(0.1).sleep();
    
    // Calls to pub1 have to be accompanied by calls to pub1_freq to keep
    // the statistics up to date.
    msg.data = false;
    pub1.publish(msg);
    pub1_freq.tick();

    // We can call updater.update whenever is convenient. It will take care
    // of rate-limiting the updates.
    updater.update();
  }

  return 0; 
}
