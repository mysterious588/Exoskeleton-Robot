#!/usr/bin/env python

#TODO plot the results

from simple_pid import PID
import rospy
from pid_.msg import motors
from std_msgs.msg import Float32

# Left Leg PID Constants
kp_ll = 1
kd_ll = 0
ki_ll = 0
# Left Thigh PID Constants
kp_lt = 1
kd_lt = 0
ki_lt = 0
# Right Thigh PID Constants
kp_rt = 1
kd_rt = 0
ki_rt = 0
# Right Leg PID Constants
kp_rl = 1
kd_rl = 0
ki_rl = 0
# topic to which we will subscribe
left_leg_topic = "left_leg_angle"
right_leg_topic = "right_leg_angle"
left_thigh_topic = "left_thigh_angle"
right_thigh_topic = "right_thigh_angle"

# motors custom message
mots = motors()

# node name
rospy.init_node('pid')

# rate in HZ
RATE = 50
rate = rospy.Rate(RATE)

# PID objects
pid_left_leg = PID(kp_ll, ki_ll, kd_ll, setpoint=30, output_limits=(-255, 255), auto_mode=True, sample_time=1 / RATE)
pid_right_leg = PID(kp_rl, ki_rl, kd_rl, setpoint=30, output_limits=(-255, 255), auto_mode=True, sample_time=1 / RATE)
pid_left_thigh = PID(kp_lt, ki_lt, kd_lt, setpoint=30, output_limits=(-255, 255), auto_mode=True, sample_time=1 / RATE)
pid_right_thigh = PID(kp_rt, ki_rt, kd_rt, setpoint=30, output_limits=(-255, 255), auto_mode=True, sample_time=1 / RATE)

# stores the array by which we will send our output to motors
output_list = {}

# robot PWM motors publishers
pub = rospy.Publisher("motors_PWM", motors, queue_size=10)


def left_leg_callback(data):
    output_list[0] = (pid_right_leg(data.data))
    # extend the motor
    if output_list[0] >= 0:
        mots.left_leg_1 = output_list[0]
        mots.left_leg_2 = 0
    else:
        # ( the opposite of extend word ) the motor
        mots.left_leg_1 = 0
        mots.left_leg_2 = output_list[0]


def right_leg_callback(data):
    output_list[1] = (pid_left_leg(data.data))
    if output_list[1] >= 0:
        mots.right_leg_1 = output_list[1]
        mots.right_leg_2 = 0
    else:
        mots.right_leg_1 = 0
        mots.right_leg_2 = output_list[1]


def left_thigh_callback(data):
    output_list[2] = (pid_left_thigh(data.data))
    if output_list[2] >= 0:
        mots.left_thigh_1 = output_list[2]
        mots.left_thigh_2 = 0
    else:
        mots.left_thigh_1 = 0
        mots.left_thigh_2 = output_list[2]


def right_thigh_callback(data):
    output_list[3] = (pid_right_thigh(data.data))
    if output_list[3] >= 0:
        mots.right_thigh_1 = output_list[3]
        mots.right_thigh_2 = 0
    else:
        mots.right_thigh_1 = 0
        mots.right_thigh_2 = output_list[3]


def update_tunings(pid, new_kp, new_ki, new_kd):
    pid.tunings((new_kp, new_ki, new_kd))


# will be called in the walking algorithm
def update_setpoint(pid, setpoint):
    pid.setpoint = setpoint


def begin_listening():
    # subscribers
    rospy.Subscriber(left_leg_topic, Float32, left_leg_callback)
    rospy.Subscriber(right_leg_topic, Float32, right_leg_callback)
    # uncomment when the MPU's upper body is finished
    # rospy.Subscriber(left_thigh_topic, Float32, left_thigh_callback)
    # rospy.Subscriber(right_thigh_topic, Float32, right_thigh_callback)
    pub.publish(mots)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    begin_listening()
