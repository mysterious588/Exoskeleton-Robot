#!/usr/bin/env python3.6

from simple_pid import PID
import rospy
from std_msgs import Float32
import serial
from matplotlib import pyplot as plt

# PID constants ##TODO: tune them
kp = 1.
kd = 0.1
ki = 0.2

plt.plot(5, 1)
plt.show()

# topic to which we will subscribe
left_leg_topic = "left_leg_angle"
right_leg_topic = "right_leg_angle"
left_thigh_topic = ""
right_thigh_topic = ""

arduino_port = '/dev/ttyACM0'

ser = serial.serial(arduino_port)

# rate in HZ
RATE = 50
rate = rospy.Rate(RATE)

# PID objects
pid_left_leg = PID(kp, ki, kd, setpoint=1, output_limits=(-255, 255), auto_mode=True, sample_time=1 / RATE)
pid_right_leg = PID(kp, ki, kd, setpoint=1, output_limits=(-255, 255), auto_mode=True, sample_time=1 / RATE)
pid_left_thigh = PID(kp, ki, kd, setpoint=1, output_limits=(-255, 255), auto_mode=True, sample_time=1 / RATE)
pid_right_thigh = PID(kp, ki, kd, setpoint=1, output_limits=(-255, 255), auto_mode=True, sample_time=1 / RATE)

# stores the array by which we will send our output to motors
output_list = {}


def left_leg_callback(data):
    output_list[0] = Float32(pid_right_leg(data.data))


def right_leg_callback(data):
    output_list[1] = Float32(pid_left_leg(data.data))


def left_thigh_callback(data):
    output_list[2] = Float32(pid_left_thigh(data.data))


def right_thigh_callback(data):
    output_list[3] = Float32(pid_right_thigh(data.data))


def update_tunings(pid, new_kp, new_ki, new_kd):
    pid.tunings((new_kp, new_ki, new_kd))


# will be called in the walking algorithm
def update_setpoint(pid, setpoint):
    pid.setpoint = setpoint


# set the sample time to the desired rate
set_sample_time(pid_left_leg, 1 / RATE)
set_sample_time(pid_right_leg, 1 / RATE)
set_sample_time(pid_left_thigh, 1 / RATE)
set_sample_time(pid_right_thigh, 1 / RATE)


def begin_listening():
    # init a node named pid
    rospy.init_node('pid')
    # subscribers
    rospy.Subscriber(left_leg_topic, Float32, left_leg_callback)
    rospy.Subscriber(right_leg_topic, Float32, right_leg_callback)
    # uncomment when the MPU's upper body is finished
    # rospy.Subscriber(left_thigh_topic, Float32, left_thigh_callback)
    # rospy.Subscriber(right_thigh_topic, Float32, right_thigh_callback)

    ser.write(output_list.encode())
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    begin_listening()
