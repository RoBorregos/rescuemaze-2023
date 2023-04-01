
#include "RosBridge.h"

// Constructor
RosBridge::RosBridge(Movement *robot, Sensors *sensors, ros::NodeHandle *nh) : robot(robot), nh(nh), sensors(sensors),
                                                                               velocity_subscriber("/cmd_vel", &RosBridge::cmdVelocityCallback, this),
                                                                               dispenser_subscriber("/dispenser", &RosBridge::dispenserCallback, this),
                                                                               test_subscriber("/testarduino", &RosBridge::testCallback, this),
                                                                               test_publisher("/testpub", &testT),
                                                                               vlx_sensor_publisher_front("/sensor/vlx/front", &vlx_sensor_msgs_front),
                                                                               tcs_sensor_publisher("/sensor/tcs", &tcs_sensor_msgs),
                                                                               limit_switch_right_publisher("/limit_switch/right", &limit_switch_right_msgs),
                                                                               limit_switch_left_publisher("/limit_switch/left", &limit_switch_left_msgs)
{

  // Node Handle
  nh->subscribe(velocity_subscriber);
  nh->subscribe(dispenser_subscriber);
  nh->subscribe(test_subscriber);

  nh->advertise(test_publisher);
  nh->advertise(vlx_sensor_publisher_front);
  nh->advertise(tcs_sensor_publisher);
  nh->advertise(limit_switch_right_publisher);
  nh->advertise(limit_switch_left_publisher);
  nh->negotiateTopics();

  // Timers
  odom_timer = millis();
  watchdog_timer = millis();
  watchdog_timer_dispenser = millis();

  // VLX message init, VLX const values
  vlx_sensor_msgs_front.radiation_type = vlx_sensor_msgs_front.INFRARED;
  vlx_sensor_msgs_front.min_range = kVLX_min;
  vlx_sensor_msgs_front.max_range = kVLX_max;
  vlx_sensor_msgs_front.field_of_view = kVLX_fov;
  vlx_sensor_msgs_front.range = 0;

  // TCs message init
  tcs_sensor_msgs.data = 'w';
  limit_switch_right_msgs.data = 0;
  limit_switch_left_msgs.data = 0;
}

// Dispenser subscriber
void RosBridge::dispenserCallback(const std_msgs::Int16 &dispenser_sign)
{
  unsigned long currentTime = millis();
  if (currentTime - watchdog_timer_dispenser > kWatchdogPeriod)
  {
    robot->dropDecider(dispenser_sign.data);
    watchdog_timer_dispenser = millis();
    watchdog_timer = millis();
  }
}

// Velocity Suscriber
void RosBridge::cmdVelocityCallback(const geometry_msgs::Twist &cmd_velocity)
{
  robot->cmdVelocity(cmd_velocity.linear.x, cmd_velocity.linear.y, cmd_velocity.angular.z);
  watchdog_timer = millis();
}

void RosBridge::testCallback(const std_msgs::String &test_msg)
{
  nh->loginfo("Data detected test");
  test_publisher.publish(&test_msg);
}

void RosBridge::watchdog()
{
  if ((millis() - watchdog_timer) > kWatchdogPeriod)
  {
    robot->stop();
    watchdog_timer = millis();
  }
}

void RosBridge::publishVLX()
{
  // VLX sensor data
  vlx_sensor_msgs_front.range = sensors->getVLXInfo(vlx_front);
  vlx_sensor_publisher_front.publish(&vlx_sensor_msgs_front);
}


void RosBridge::publishLimitSwitches()
{
  int rightSwitch = 0;
  int leftSwitch = 0;

  rightSwitch = robot->rightLimitSwitch();
  leftSwitch = robot->leftLimitSwitch();
  
  if (rightSwitch == 1)
  {
    limit_switch_right_msgs.data = 1;
    limit_switch_right_publisher.publish(&limit_switch_right_msgs);
  } else {
    limit_switch_right_msgs.data = 0;
    limit_switch_right_publisher.publish(&limit_switch_right_msgs);
  }
    if (leftSwitch == 1)
  {
    limit_switch_left_msgs.data = 1;
    limit_switch_left_publisher.publish(&limit_switch_left_msgs);
  } else {
    limit_switch_left_msgs.data = 0;
    limit_switch_left_publisher.publish(&limit_switch_left_msgs);
  }
}


void RosBridge::publish()
{
  unsigned long currentTime = millis();
  if ((currentTime - odom_timer) > kOdomPeriod)
  {
    odom_timer = currentTime;

    // TCS sensor data
    tcs_sensor_msgs.data = sensors->getTCSInfo();
    tcs_sensor_publisher.publish(&tcs_sensor_msgs);
    
    publishVLX();
    publishLimitSwitches();
  }
}

// Run
void RosBridge::run()
{
  while (1)
  {
    watchdog();
    publish();
    nh->spinOnce();
  }
}
