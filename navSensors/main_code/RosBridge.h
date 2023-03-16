
#ifndef RosBridge_h
#define RosBridge_h

#include <ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <stdint.h>
#include <math.h>
#include <Arduino.h>

#include "Movement.h"
#include "Sensors.h"


class RosBridge{
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

    // Subscriber to make tests
    void testCallback(const std_msgs::String &test_msg);

    // Verify it is still receiving velocity commands.
    void watchdog();

    // Encoders Publisher
    // Publish data to topics.
    void publish();

    void publishVLX();

    Movement *robot;
    Sensors *sensors;

    // Node.
    ros::NodeHandle *nh;

    // Suscribers
    
    ros::Subscriber<geometry_msgs::Twist, RosBridge> velocity_subscriber;
    ros::Subscriber<std_msgs::Int16, RosBridge> dispenser_subscriber;

    ros::Subscriber<std_msgs::String, RosBridge> test_subscriber;

    static constexpr uint16_t kWatchdogPeriod = 500;
    
    // Publishers

    ros::Publisher vlx_sensor_publisher_left;
    ros::Publisher vlx_sensor_publisher_right;
    ros::Publisher vlx_sensor_publisher_front;
    ros::Publisher vlx_sensor_publisher_back;

    ros::Publisher tcs_sensor_publisher;
    ros::Publisher test_publisher;

    // Messages
    sensor_msgs::Range vlx_sensor_msgs_left;  // TOF sensors
    sensor_msgs::Range vlx_sensor_msgs_right; // TOF sensors
    sensor_msgs::Range vlx_sensor_msgs_front; // TOF sensors
    sensor_msgs::Range vlx_sensor_msgs_back;  // TOF sensors

    std_msgs::Char tcs_sensor_msgs;          // Color sensor
    std_msgs::String testT;

    static constexpr uint8_t kOdomPeriod = 40;

    // Timers.
    unsigned long odom_timer = 0;
    unsigned long watchdog_timer = 0;
    unsigned long watchdog_timer_dispenser = 0;

    // Motor and Sensor constants
    // Sensor
    static constexpr int kBNOData = 10;
    static constexpr int kVLXData = 6;
    static constexpr int kMLXData = 2;
    static constexpr int kTCSData = 1;
    static constexpr int kSensorData = kBNOData + kVLXData + kMLXData + kTCSData;

    // VLX
    const float kVLX_fov = (45 * 3.1416) / 180;
    const float kVLX_min = 0.03;
    const float kVLX_max = 3;

    // Motor.
    const int kMotorCount = 4;
};

#endif
