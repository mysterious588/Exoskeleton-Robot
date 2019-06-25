#include <MPU.h>
#include <Wire.h> //must be included

MPU mpu1(1);//0x68 mpu
MPU mpu2(2);//0x69 mpu

void setup() {
  //setup the MPUs
  mpu1.setup();
  mpu2.setup();
}

void loop() {
  //calculate new angles for each loop
  mpu1.calculateAngles();
  mpu2.calculateAngles();

  //get the readings for the x-axis
  double x1 = mpu1.getX();
  double x2 = mpu2.getX();

  //print the results
  Serial.print("readings from the first mpu: "); Serial.println(x1);
  Serial.print("readings from the second mpu: "); Serial.println(x2);
}
