#include "Sensors.h"

////////////////////////////////////////Constructor//////////////////////////////////////////////////////
Sensors::Sensors(BNO *bno) : bno_(bno){
  initSensors();
}
    
////////////////////////////////////////Initialization/////////////////////////////////////////////////////////
void Sensors::initSensors(){

  for(int i=0; i<kVLXCount; i++){
    vlx_[i].setMux(kMuxVLX[i]);
  }

  for(int i=0; i<kMLXCount; i++){
    mlx_[i].setMux(kMuxMLX[i]);
  }

  tcs_.setMux(kMuxTCS);

  Wire.begin();

  // BNO init (Not needed, because initialized before sending pointer)

  //TCS init
  tcs_.init();  

  // VLX init
  for(int i=0; i<kVLXCount; i++){
    vlx_[i].init();
  }

  // MLX init
  for(int i=0; i<kMLXCount; i++){
    mlx_[i].init();
  }
}

////////////////////////////////////////Sensor Methods/////////////////////////////////////////////////////////
void Sensors::printInfo(){
  
  bno_->anglesInfo();

  for(int i=0; i<kVLXCount; i++){
    Serial.print("VLX sensor ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(float(getVLXInfo(i)),4);
  }

  for(int i=0; i<kMLXCount; i++){
    Serial.print("MLX sensor ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(getMLXInfo(i));
  }

  //Serial.print("TCS sensor  ");
  tcs_.printRGB();
}

float Sensors::getVLXInfo(int posVLX){
  return vlx_[posVLX].getDistance();
}

float Sensors::getMLXInfo(int posMLX){
  return mlx_[posMLX].getTemp();
}

float Sensors::getQuatX(){
  return bno_->getQuat_x();
}

float Sensors::getQuatY(){
  return bno_->getQuat_y();
}

float Sensors::getQuatZ(){
  return bno_->getQuat_z();
}

float Sensors::getQuatW(){
  return bno_->getQuat_w();
}

float Sensors::getAngVelX(){
  return bno_->getAngVel_x();
}

float Sensors::getAngVelY(){
  return bno_->getAngVel_y();
}

float Sensors::getAngVelZ(){
  return bno_->getAngVel_z();
}

float Sensors::getLinAccX(){
  return bno_->getLinAcc_x();
}

float Sensors::getLinAccY(){
  return bno_->getLinAcc_y();
}

float Sensors::getLinAccZ(){
  return bno_->getLinAcc_z();
}

float Sensors::getAngleX(){
  return bno_->getAngleX();
}

float Sensors::getAngleY(){
  return bno_->getAngleY();
}

float Sensors::getAngleZ(){
  return bno_->getAngleZ();
}

char Sensors::getTCSInfo(){
  return tcs_.getColor();
}
