#include <Arduino.h>
#include <Wire.h>
#include<MPU.h>
#define RAD_TO_DEG 180/3.1415
int address;
MPU::MPU(int n)
{
if (n == 1)
    address = 0x68;
    else if (n == 2)
        address = 0x69;
}

MPU::~MPU()
{
    //dtor
}
double MPU::getX(){
Serial.println("Here");
	return this->x;
}
double MPU::getY(){
	return this->getY();
}
double MPU::getZ(){
	return this->getZ();
}
void MPU::setX(double x){
	this->x = x;
}
void MPU::setY(double y){
	this->y = y;
}
void MPU::setZ(double z){
	this->z = z;
}
void MPU::setup(){
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}
void MPU::calculateAngles(){
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
  Wire.beginTransmission(address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  int xAng = map(AcX, 265, 402, -90, 90);
  int yAng = map(AcY, 265, 402, -90, 90);
  int zAng = map(AcZ, 265, 402, -90, 90);

  this->setX(RAD_TO_DEG * (atan2(-yAng, -zAng) + 3.1415));
  this->setY(RAD_TO_DEG * (atan2(-xAng, -zAng) + 3.1415));
  this->setZ(RAD_TO_DEG * (atan2(-yAng, -xAng) + 3.1415));

}
