
#include "RosBridge.h"

// Constructor
RosBridge::RosBridge(Movement *robot, Sensors *sensors, ros::NodeHandle *nh) : robot(robot), nh(nh), sensors(sensors),
                                                                               velocity_subscriber("/cmd_vel", &RosBridge::cmdVelocityCallback, this),
                                                                               dispenser_subscriber("/dispenser", &RosBridge::dispenserCallback, this),
                                                                               test_subscriber("/testarduino", &RosBridge::testCallback, this),
                                                                               dist_subscriber("/dist_walls", &RosBridge::updateDistLidarCallback, this),
                                                                               cmd_movement_subscriber("/unit_movement", &RosBridge::cmdMovementCallback, this),
                                                                               test_publisher("/testpub", &testT),
                                                                               vlx_sensor_publisher_front("/sensor/vlx/front", &vlx_sensor_msgs_front),
                                                                               vlx_sensor_publisher_right("/sensor/vlx/right", &vlx_sensor_msgs_right),
                                                                               vlx_sensor_publisher_left("/sensor/vlx/left", &vlx_sensor_msgs_left),
                                                                               req_dist_publisher("/dist_request", &dist_req),
                                                                               tcs_sensor_publisher("/sensor/tcs", &tcs_sensor_msgs),
                                                                               limit_switch_right_publisher("/limit_switch/right", &limit_switch_right_msgs),
                                                                               limit_switch_left_publisher("/limit_switch/left", &limit_switch_left_msgs),
                                                                               cmd_movement_publisher("/control_feedback", &cmd_movement_response)
{

  // Node Handle

  if (!CK::onlyCmdMovement)
  {
    nh->subscribe(velocity_subscriber);
    nh->subscribe(test_subscriber);

    nh->advertise(test_publisher);
    nh->advertise(tcs_sensor_publisher);
    nh->advertise(limit_switch_right_publisher);
    nh->advertise(limit_switch_left_publisher);
    nh->advertise(test_publisher);
    nh->advertise(vlx_sensor_publisher_left);
  }
  else
  {
    nh->subscribe(cmd_movement_subscriber);
    nh->advertise(cmd_movement_publisher);
  }

  nh->subscribe(velocity_subscriber);

  nh->advertise(vlx_sensor_publisher_front);
  nh->advertise(vlx_sensor_publisher_right);
  nh->subscribe(dispenser_subscriber);

  // Subscriber and publisher for distance to walls
  nh->subscribe(dist_subscriber);
  nh->advertise(req_dist_publisher);

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
void RosBridge::dispenserCallback(const std_msgs::Int8 &dispenser_sign)
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

void RosBridge::cmdMovementCallback(const std_msgs::Int8 &cmd_movement_req)
{
  double response = robot->cmdMovement(cmd_movement_req.data);

  // Transform response to algorithm response

  if (response == 0){
    // do nothing
  } else if (response == 1){
    
  }
  
  cmd_movement_response.data = response;
  cmd_movement_publisher.publish(&cmd_movement_response);
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

void RosBridge::updateDistLidarCallback(const geometry_msgs::Quaternion &dist)
{
  sensors->updateDistLidar(dist.x, dist.y, dist.z, dist.w);
}

void RosBridge::updateDistLidar()
{
  req_dist_publisher.publish(&dist_req);
  nh->spinOnce();
}

void RosBridge::publishVLX()
{
  // VLX sensor data
  vlx_sensor_msgs_front.range = sensors->getVLXInfo(vlx_front);
  vlx_sensor_publisher_front.publish(&vlx_sensor_msgs_front);

  vlx_sensor_msgs_right.range = sensors->getVLXInfo(vlx_right);
  vlx_sensor_publisher_right.publish(&vlx_sensor_msgs_right);

  vlx_sensor_msgs_left.range = sensors->getVLXInfo(vlx_left);
  vlx_sensor_publisher_left.publish(&vlx_sensor_msgs_left);

}

void RosBridge::publishLimitSwitches()
{
  int rightSwitch = 0;
  int leftSwitch = 0;

  sensors->getLimitSwitches(rightSwitch, leftSwitch);

  if (rightSwitch == 1)
  {
    limit_switch_right_msgs.data = 1;
    limit_switch_right_publisher.publish(&limit_switch_right_msgs);
  }
  else
  {
    limit_switch_right_msgs.data = 0;
    limit_switch_right_publisher.publish(&limit_switch_right_msgs);
  }
  if (leftSwitch == 1)
  {
    limit_switch_left_msgs.data = 1;
    limit_switch_left_publisher.publish(&limit_switch_left_msgs);
  }
  else
  {
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

    if (!CK::onlyCmdMovement)
    {
      // TCS sensor data
      // tcs_sensor_msgs.data = sensors->getTCSInfo();
      // tcs_sensor_publisher.publish(&tcs_sensor_msgs);

      publishVLX();
      // publishLimitSwitches();
    }
  }
}

// Run
void RosBridge::run()
{
  while (1)
  {
    // watchdog(); // use with cmd_vel
    publish();
    nh->spinOnce();
  }
}

void RosBridge::rosBridgeTest()
{
  while (true)
  {
    long int initialT = millis();
    nh->loginfo("Requesting lidar data");
    updateDistLidar();
    nh->loginfo("Request flidar finished in ");
    logNumber(millis() - initialT);
    nh->loginfo(" ms");
    double front, back, left, right;
    sensors->getLidarDistances(front, back, left, right);
    nh->loginfo("Distances obtained (front, back, left, right):");
    logDist(front, back, left, right);

    delay(100);
  }
}

// Helper function to log numbers.
void RosBridge::logNumber(double number){
  String str = String(number);
  const char* message = str.c_str();
  nh->loginfo(message);
}

// Helper function to log distances.
void RosBridge::logDist(double front, double back, double left, double right){
  String all = String(front) + " " + String(back) + " " + String(left) + " " + String(right);
  const char* message = all.c_str();
  nh->loginfo(message);
}