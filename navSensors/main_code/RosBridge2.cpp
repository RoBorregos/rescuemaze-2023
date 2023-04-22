#include "RosBridge2.h"

//////////////////////////////////Constructor//////////////////////////////////////
RosBridge2::RosBridge2(Movement *robot, Sensors *sensors, BNO *bno) : robot_(robot), sensors_(sensors), bno_(bno)
{

  // Timers
  odom_timer_ = millis();
  watchdog_timer_ = millis();
}

// Movement Suscriber
void RosBridge2::cmdMovementCallback(int move)
{
  state_ = -1; // Stauts code to indicate that the robot is moving

  double response = robot_->cmdMovement(move, 1);

  if (response == -2)
    return response;

  if (response == 0)
  {
    // do nothing
  }
  else if (response == 1)
  {
    // check color
    char color = sensors_->getTCSInfo();

    if (color == 'A')
    {
      // wait 5 seconds and return result
      delay(5000);
      response = 2;
    }
    else if (color == 'P')
    {
      // checkpoint detected
      response = 3;
    }
  }
  else // went through ramp
  {
    if (response > CK::kRampDt)
    {
      // ramp detected
      response = 4;
    }
    else
    {
      // ramp not detected
      response = 2;
    }
  }
  state_ = response;
}

void RosBridge2::updateDistLidar(float front, float back, float left, float right)
{
  sensors_->updateDistLidar(front, back, left, right);
}

void RosBridge2::callDispenser(int victims)
{
  robot_->cmdMovement(7, victims);
}

void RosBridge2::executeCommand(uint8_t packet_size, uint8_t command, uint8_t *buffer)
{
  switch (command)
  {
  case 0x00: // Baud
    if (packet_size == 1)
    { // Check packet size
      uint32_t baud[] = {57600};
      writeSerial(true, (uint8_t *)baud, sizeof(baud));
    }
    break;
  case 0x01: // Get VLX
    if (packet_size == 1)
    { // Check packet size
      float data[] = {sensors_->getVLXInfo(vlx_front), sensors_->getVLXInfo(vlx_right), sensors_->getVLXInfo(vlx_left)};
      writeSerial(true, (uint8_t *)data, sizeof(data));
    }
    break;
  case 0x02: // get_goal_state
    if (packet_size == 1)
    { // Check packet size
      float data[] = {state_};
      writeSerial(true, (uint8_t *)data, sizeof(data));
    }
    break;
  case 0x03: // set_goal
    if (packet_size == 5)
    { // Check packet size
      int move;
      memcpy(&move, buffer, sizeof(move));
      cmdMovementCallback(move);
      writeSerial(true, nullptr, 0);
    }
    break;
  case 0x04: // send_lidar
    if (packet_size == 17)
    { // Check packet size
      float front, back, left, right;
      // Copy data from buffer to variables
      memcpy(&front, buffer, sizeof(front));
      memcpy(&back, buffer + sizeof(front), sizeof(back));
      memcpy(&left, buffer + sizeof(front) + sizeof(back), sizeof(left));
      memcpy(&right, buffer + sizeof(front) + sizeof(back) + sizeof(left), sizeof(right));
      updateDistLidar(front, back, left, right);
      writeSerial(true, nullptr, 0);
    }
    break;
  case 0x05: // send_victims
    if (packet_size == 5)
    { // Check packet size
      int victims;
      // Copy data from buffer to variables
      memcpy(&victims, buffer, sizeof(victims));
      writeSerial(true, nullptr, 0);
      callDispenser(victims);
    }
    break;
  case 0x06: // get_start_state
    if (packet_size == 1)
    { // Check packet size
      bool data[] = {sensors_->readMotorInit()};
      if (!sensors_->readMotorInit())
        robot_->resetMovement();
      writeSerial(true, (uint8_t *)data, sizeof(data));
    }
    break;
  case 0x07: // get_lidar
    if (packet_size == 1)
    { // Check packet size
      float data[] = {sensors_->wallDistances[0], sensors_->wallDistances[1], sensors_->wallDistances[2], sensors_->wallDistances[3]};
      writeSerial(true, (uint8_t *)data, sizeof(data));
    }
    break;
  case 0x08: // get goal
    if (packet_size == 1)
    {
      int data[] = {this->goal};
      writeSerial(true, (uint8_t *)data, sizeof(data));
    }
    break;
  case 0x09: // Get IMU
    if (packet_size == 1)
    { // Check packet size
      // Yaw, yaw vel, getXaccel, getYaccel, getZaccel
      float data[] = {bno_->getYaw(), bno_->getYawVel(), bno_->getXAccel(), bno_->getYAccel(), bno_->getZAccel()};
      writeSerial(true, (uint8_t *)data, sizeof(data));
    }
    break;
  case 0x0A: // Get if lidar is being used
    if (packet_size == 1)
    { // Check packet size

      bool data[] = {sensors_->usingLidar};
      writeSerial(true, (uint8_t *)data, sizeof(data));
    }
    break;
  default:
    break;
  }
}

void RosBridge2::writeSerial(bool success, uint8_t *payload, int elements)
{
  uint8_t ack = success ? 0x00 : 0x01;
  Serial.write(0xFF);
  Serial.write(0xAA);
  Serial.write(sizeof(uint8_t) * elements + 1); // Packet size
  Serial.write(ack);                            // ACK

  // Send payload bytes
  for (size_t i = 0; i < elements; i++)
  {
    Serial.write(payload[i]);
  }

  Serial.write(0x00); // Footer
  Serial.flush();
}
void RosBridge2::readSerial()
{
  static uint8_t buffer[24];
  static uint8_t index = 0;
  static uint8_t packet_size = 0;
  static uint8_t command = 0;
  static uint8_t check_sum = 0;

  while (Serial.available())
  {
    buffer[index++] = Serial.read();

    // Check packet header
    if (index == 1 && buffer[0] != 0xFF)
    {
      index = 0;
      packet_size = 0;
      command = 0;
    }
    if (index == 2 && buffer[1] != 0xAA)
    {
      packet_size = 0;
      command = 0;
      index = 0;
    }

    // Read packet size and command
    if (index == 4)
    {
      packet_size = buffer[2];
      command = buffer[3];
    }

    // Check if the entire packet has been received
    if (index == 3 + (packet_size) + 1)
    {
      check_sum = buffer[index - 1];
      if (check_sum != command + 1)
      {
        // Checksum error
        index = 0;
        packet_size = 0;
        command = 0;
        continue;
      }
      // Execute the command
      executeCommand(packet_size, command, &buffer[4]);

      // Reset index and packet_size
      index = 0;
      packet_size = 0;
    }
  }
}

//////////////////////////////////Run//////////////////////////////////////
void RosBridge2::run()
{
  while (1)
  {
    readSerial();
    if ((millis() - watchdog_timer_) > kWatchdogPeriod)
    {
      // Decide to do something after ktime has passed without receiving a command
    }
  }
}

void RosBridge2::readOnce()
{
  readSerial();
}