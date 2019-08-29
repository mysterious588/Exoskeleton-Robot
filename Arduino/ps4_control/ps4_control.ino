#include <ros.h>
#include <sensor_msgs/Joy.h>

char L1, R1, L2, R2;
char squ, x, circle, tri;
const char MOTORS[8] = {1, 2, 3, 4, 5, 6, 7, 8}; //motor pins

ros::NodeHandle_<ArduinoHardware, 5, 5, 128, 512> nh;

void messageCb(const sensor_msgs::Joy& msg) {
  R1 = msg.buttons[5];
  R2 = msg.buttons[7];
  L1 = msg.buttons[4];
  L2 = msg.buttons[6];
  squ = msg.buttons[3];
  x  = msg.buttons[-1]; // detect once again
  circle = msg.buttons[1];
  tri = msg.buttons[2];

  if (L1) {
    digitalWrite(MOTORS[0], 1);
    digitalWrite(MOTORS[1], 0);
  }
  else if (L2) {
    digitalWrite(MOTORS[0], 0);
    digitalWrite(MOTORS[1], 1);
  }
  else if (R1) {
    digitalWrite(MOTORS[2], 1);
    digitalWrite(MOTORS[3], 0);
  }
  else if (R2) {
    digitalWrite(MOTORS[2], 0);
    digitalWrite(MOTORS[3], 1);
  }
  else if (x) {
    digitalWrite(MOTORS[4], 1);
    digitalWrite(MOTORS[5], 0);
  }
  else if (tri) {
    digitalWrite(MOTORS[4], 0);
    digitalWrite(MOTORS[5], 1);
  }
  else if (circle) {
    digitalWrite(MOTORS[6], 1);
    digitalWrite(MOTORS[7], 0);
  }
  else if (squ) {
    digitalWrite(MOTORS[6], 0);
    digitalWrite(MOTORS[7], 1);
  }
  else
    for (char i = 0; i < 8; i++)
      digitalWrite(MOTORS[i], 0);

}

ros::Subscriber<sensor_msgs::Joy> sub("joy", messageCb);


void setup() {
  for (char i = 0; i < 8; i++)
    pinMode(MOTORS[i], 1);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
