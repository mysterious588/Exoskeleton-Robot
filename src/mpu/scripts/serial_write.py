#!/usr/bin/env python
import serial
import rospy
from pid.msg import motors
motors_publisher = rospy.Publisher("",)
ser = serial.Serial('/dev/ttyACM1', 9600)
arr = {1., 2., 5.}

int_encode = b'%d' % 5


def callback(data):
    while 1:
        for i in range(8):
            ser.write(b"%d" % i)
            if (ser.in_waiting > 0):
                print(ser.readline())


if __name__ == '__main__':
    callback(arr)
