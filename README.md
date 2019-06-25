# Exoskeleton-Robot
Exoskeleton is a wearable device that allows paraplegic patients to control their legs 

## Installation
1- Make sure you have ROS Kinetic installed in your device.

Follow this link for instructions http://wiki.ros.org/kinetic/Installation

2- clone the repository
```bash 
git clone https://github.com/mysterious588/Exoskeleton-Robot
```
3- init your workspace
```bash
catkin_make
```
## prerequisites
Some library has to be installed & enabling the I2C bus for the Raspberry Pi is necessary

check our tutoral: https://github.com/mysterious588/Exoskeleton-Robot/tree/master/old%20libraries/i2c%20raspberry%20pi

## MPUs
After installation you can run the MPU scripts
Make sure you have the MPUs connected to your Raspberry pi

All of the MPUs will be connected the same way excpet the AD0 Pin which controls the address.

The script pulls three MPUs' AD0 Pin high to make the address 0x69 & then reads the 0x68 address of the remaining MPU, This opens the door to use multiple MPUs in the same I2C bus.

Vcc >> 5v or 3.3V (MPU6050 has a built in voltage regulator)

Gnd >> Gnd

SCL >> SCL

SDA >> SDA

Pin 21 >> Left Leg MPU

Pin 22 >> Left Thigh MPU

Pin 23 >> Right Leg MPU

Pin 24 >> Right Thigh MPU

Make sure you have your workspace sourced
```bash
source Exo_workspace/devel/setup.bash
```

Then run the following command

```bash
rosrun mpu MPU.py
```
