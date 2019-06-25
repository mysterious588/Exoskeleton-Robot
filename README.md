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
## MPUs
After installation you can run the mpu scripts
Make sure you have the MPU Connected to your Raspberry pi

Vcc >> 5v

Gnd >> Gnd

SCL >> SCL

SDA >> SDA

Make sure you have your workspace sourced
```bash
source Exo_workspace/devel/setup.bash
```

Then run the following command

```bash
rosrun mpu lower_left_mpu.py
```
