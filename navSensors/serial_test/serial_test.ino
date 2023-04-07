#include <ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include "RosBridgeTest.h"

ros::NodeHandle nh;
void setup()
{
    Serial.begin(57600);

    nh.initNode();

    while (!nh.connected())
    {
        nh.spinOnce();
    }

    RosBridge rosbridge(&nh);

    rosbridge.run();

}

void loop()
{
  nh.loginfo("Loop log");
  nh.loginfo("Loop log");
  nh.loginfo("Loop log");
  nh.loginfo("Loop log");
  nh.loginfo("Loop log");
  nh.loginfo("Loop log");
  nh.loginfo("Loop log");
  nh.loginfo("Loop log");
  nh.loginfo("Loop log");
  nh.loginfo("Loop log");
  nh.loginfo("Loop log");
  nh.loginfo("Loop log");

}
