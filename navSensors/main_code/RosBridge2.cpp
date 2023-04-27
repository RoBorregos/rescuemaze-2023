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
  robot_->rearrangeAngle(2.0);

  if (response == -2)
    state_= response;

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
      response = 5;
    }
    else if (abs(response) > CK::kRampDt)
    {
      response = 4;
    }
    else if (response > 0)
    {
      // Went up stairs
      response = 6;
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

void RosBridge2::updateDistLidar(float front)
{
  sensors_->updateDistLidar(front);
}

void RosBridge2::advanceXMeters(float meters)
{
  state_ = -1;
  robot_->advanceXMeters(meters, 1);
  sensors_->logActive("Adv x M", true, 0);
  state_ = 1;
}

void RosBridge2::callDispenser(int victims)
{
  robot_->cmdMovement(7, victims);
}

void RosBridge2::executeCommand(uint8_t packet_size, uint8_t command, uint8_t *buffer)
{
  lastInstruction = millis();
  cmdCounterT += 1;
  sensors_->logActive("Cmds R: " + String(cmdCounterT), true, 0, 0, true);
  // sensors_->logActive("ReadSerial Count: " + String(countReadSerial), true, 0, 5, true);

  sensors_->logActive("Gs: " + String(cmdCounter[1]), true, 0, 2, true);
  sensors_->logActive("Sg: " + String(cmdCounter[2]), true, 0, 3, true);
  sensors_->logActive("Sl: " + String(cmdCounter[3]), true, 0, 4, true);
  // sensors_->logActive("Sv: " + String(cmdCounter[4]), true, 0, 5, true);
  // sensors_->logActive("Ss: " + String(cmdCounter[5]), true, 0, 6, true);
  //sensors_->logActive("Gl: " + String(cmdCounter[6]), true, 0, 7, true);
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
    {
      cmdCounter[0]++;
      // Check packet size
      float data[] = {sensors_->getVLXInfo(vlx_front), sensors_->getVLXInfo(vlx_right), sensors_->getVLXInfo(vlx_left)};
      writeSerial(true, (uint8_t *)data, sizeof(data));
    }
    break;
  case 0x02: // get_goal_state
    if (packet_size == 1)
    { // Check packet size
      cmdCounter[1]++;
      float data[] = {state_};
      writeSerial(true, (uint8_t *)data, sizeof(data));
    }
    break;
  case 0x03: // set_goal
    if (packet_size == 5)
    { // Check packet size
      cmdCounter[2]++;
      int move;
      memcpy(&move, buffer, sizeof(move));
      sensors_->logActive("Move: " + String(move), true, 0, 7, true);
      cmdMovementCallback(move);
      writeSerial(true, nullptr, 0);
    }
    break;
  case 0x04: // send_lidar
    // if (packet_size == 17)
    if (packet_size == 5)
    { // Check packet size
      cmdCounter[3]++;
      float front;
      // Copy data from buffer to variables
      memcpy(&front, buffer, sizeof(front));
      // memcpy(&right, buffer + sizeof(front), sizeof(back));
      // memcpy(&left, buffer + sizeof(front) + sizeof(back), sizeof(left));
      // memcpy(&back, buffer + sizeof(front) + sizeof(back) + sizeof(left), sizeof(right));
      updateDistLidar(front);
      // updateDistLidar(front, back, left, right);
      writeSerial(true, nullptr, 0);
    }
    break;
  case 0x05: // send_victims
    if (packet_size == 5)
    { // Check packet size
      int victims;
      cmdCounter[4]++;
      // Copy data from buffer to variables
      memcpy(&victims, buffer, sizeof(victims));
      callDispenser(victims);
      writeSerial(true, nullptr, 0);
    }
    break;
  case 0x06: // get_start_state
    if (packet_size == 1)
    { // Check packet size
      cmdCounter[5]++;
      bool data[] = {sensors_->readMotorInit()};
      if (!sensors_->readMotorInit())
        robot_->resetMovement();
      writeSerial(true, (uint8_t *)data, sizeof(data));
    }
    break;
  case 0x07: // get_lidar
    if (packet_size == 1)
    { // Check packet size
      cmdCounter[6]++;
      float data[] = {sensors_->wallDistances[0], sensors_->wallDistances[1], sensors_->wallDistances[2], sensors_->wallDistances[3]};
      writeSerial(true, (uint8_t *)data, sizeof(data));
    }
    break;
  case 0x08: // get goal
    if (packet_size == 1)
    {
      cmdCounter[7]++;
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
  case 0x0B: // Move specific distance
    if (packet_size == 5)
    { // Check packet size
      float distance;
      memcpy(&distance, buffer, sizeof(distance));
      sensors_->logActive("Adv abs: " + String(distance), true, 0, 6, true);
      robot_->advanceXMetersAbs(distance, 1);
      writeSerial(true, nullptr, 0);
    }
    break;
  default:
    break;
  }
  sensors_->logActive("Cmds exec: " + String(cmdCounterT), true, 0, 1, true);
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
  // countReadSerial++;
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

bool RosBridge2::readLidar()
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
      if (command == 0x04)
        return true;

      // Reset index and packet_size
      index = 0;
      packet_size = 0;
    }
  }
  return false;
}

//////////////////////////////////Run//////////////////////////////////////
void RosBridge2::run()
{
  lastInstruction = millis();
  while (1)
  {
    if (millis() - lastInstruction > kOnlyArduinoTimer){
      // Return to void loop in main_code, which should run exploreFollowerWall2() or similar.
      return;
    }
    readSerial();
  }
}

void RosBridge2::readOnce()
{
  readSerial();
}

void RosBridge2::readUntilLidar()
{
  bool lidarRead = false;
  while (!lidarRead)
  {
    lidarRead = readLidar();
  }
}