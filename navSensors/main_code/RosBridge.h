
#ifndef RosBridge_h
#define RosBridge_h

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

#include <stdint.h>
#include <math.h>
#include <Arduino.h>

#include "Movement.h"
#include "Sensors.h"
#include "CommonK.h"

class RosBridge
{
  friend class GeneralChecks;

public:
  // Constructor
  RosBridge(Movement *robot, Sensors *sensors, ros::NodeHandle *nh);

  // Run
  // Calls watchdog and publish.
  void run();

private:
  // Dispenser Subscriber
  // Receives dispenser drop position
  void dispenserCallback(const std_msgs::Int16 &dispenser_sign);

  // Velocity Subscriber
  // Receives velocity commands.
  void cmdVelocityCallback(const geometry_msgs::Twist &cmdvel);

  // Movement Subscriber
  // Receives movement commands.
  void cmdMovementCallback(const std_msgs::Int16 &cmd_movement);

  // Subscriber to make tests
  void testCallback(const std_msgs::String &test_msg);

  // Verify it is still receiving velocity commands.
  void watchdog();

  // Encoders Publisher
  // Publish data to topics.
  void publish();

  void publishVLX();

  void publishLimitSwitches();

  Movement *robot;
  Sensors *sensors;

  // Node.
  ros::NodeHandle *nh;

  // Suscribers

  ros::Subscriber<geometry_msgs::Twist, RosBridge> velocity_subscriber;
  ros::Subscriber<std_msgs::Int16, RosBridge> cmd_movement_subscriber;
  ros::Subscriber<std_msgs::Int16, RosBridge> dispenser_subscriber;
  ros::Subscriber<std_msgs::String, RosBridge> test_subscriber;

  static constexpr uint16_t kWatchdogPeriod = 500;

  // Publishers
  ros::Publisher cmd_movement_publisher;
  ros::Publisher vlx_sensor_publisher_front;
  ros::Publisher tcs_sensor_publisher;
  ros::Publisher limit_switch_right_publisher; // Right limit switch
  ros::Publisher limit_switch_left_publisher;  // Left limit switch
  ros::Publisher test_publisher;

  // Messages
  sensor_msgs::Range vlx_sensor_msgs_front; // TOF sensor
  std_msgs::Char tcs_sensor_msgs;           // Color sensor
  std_msgs::String testT;
  std_msgs::Int16 limit_switch_right_msgs; // Right limit switch
  std_msgs::Int16 limit_switch_left_msgs;  // Right limit switch

  std_msgs::Int16 cmd_movement_input;  // Right limit switch
  std_msgs::Int16 cmd_movement_response;  // Right limit switch

  static constexpr uint8_t kOdomPeriod = 40;

  // Timers.
  unsigned long odom_timer = 0;
  unsigned long watchdog_timer = 0;
  unsigned long watchdog_timer_dispenser = 0;

  // Motor and Sensor constants

  // VLX
  const float kVLX_fov = (45 * 3.1416) / 180;
  const float kVLX_min = 0.03;
  const float kVLX_max = 3;

  // Motor.
  const int kMotorCount = 4;
};

#endif
