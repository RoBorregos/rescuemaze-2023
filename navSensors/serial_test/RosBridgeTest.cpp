
#include "RosBridgeTest.h"

// Constructor
RosBridge::RosBridge(ros::NodeHandle *nh) : nh(nh),
                                            velocity_subscriber("/cmd_vel", &RosBridge::cmdVelocityCallback, this),
                                            dispenser_subscriber("/dispenser", &RosBridge::dispenserCallback, this),
                                            test_subscriber("/testarduino", &RosBridge::testCallback, this),
                                            test_publisher("/testpub", &testT),
                                            vlx_sensor_publisher_front("/sensor/vlx/front", &vlx_sensor_msgs_front),
                                            tcs_sensor_publisher("/sensor/tcs", &tcs_sensor_msgs),
                                            limit_switch_right_publisher("/limit_switch/right", &limit_switch_right_msgs),
                                            limit_switch_left_publisher("/limit_switch/left", &limit_switch_left_msgs),
                                            init_robot_publisher("/robot_init", &init_robot_msg)
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
  nh->advertise(init_robot_publisher);
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
  init_robot_msg.data = 0;
}

// Dispenser subscriber
void RosBridge::dispenserCallback(const std_msgs::Int16 &dispenser_sign)
{
  unsigned long currentTime = millis();
  if (currentTime - watchdog_timer_dispenser > kWatchdogPeriod)
  {
    watchdog_timer_dispenser = millis();
    watchdog_timer = millis();
  }
}

// Velocity Suscriber
void RosBridge::cmdVelocityCallback(const geometry_msgs::Twist &cmd_velocity)
{
  watchdog_timer = millis();
}

void RosBridge::testCallback(const std_msgs::String &test_msg)
{
  if (true)
  {
    char log_msg[20];
    char result[8];
    double rpms[kMotorCount];

    nh->loginfo(log_msg);
    nh->loginfo(log_msg);
    nh->loginfo(log_msg);
    nh->loginfo(log_msg);
  }
  nh->loginfo("Data detected test");
  test_publisher.publish(&test_msg);
}

void RosBridge::watchdog()
{
  if ((millis() - watchdog_timer) > kWatchdogPeriod)
  {
    watchdog_timer = millis();
  }
}

void RosBridge::publishVLX()
{
  // VLX sensor data
  vlx_sensor_publisher_front.publish(&vlx_sensor_msgs_front);

  nh->loginfo("Publish vlx");
  nh->loginfo("Publish vlx");
}


void RosBridge::publishLimitSwitches()
{
  int rightSwitch = 0;
  int leftSwitch = 0;
  nh->loginfo("Publish ");
  nh->loginfo("Publish ");
  
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
  nh->loginfo("Publish limit");
  nh->loginfo("Publish limit");
}


void RosBridge::publish()
{
  unsigned long currentTime = millis();
  if ((currentTime - odom_timer) > kOdomPeriod)
  {
    odom_timer = currentTime;

    // TCS sensor data
    tcs_sensor_publisher.publish(&tcs_sensor_msgs);
    
    publishVLX();
    publishLimitSwitches();
    publishInitRobot();
  }
}

// Publish init robot
void RosBridge::publishInitRobot()
{
  int val = digitalRead(22);
  if (val == HIGH){
    init_robot_msg.data = 1;
  } else {
    init_robot_msg.data = 0;
  }
  init_robot_publisher.publish(&init_robot_msg);

  nh->loginfo("Publish init");
  nh->loginfo("Publish init");
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
