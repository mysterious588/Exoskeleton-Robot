#!/usr/bin/env python
import serial
import rospy

ser = serial.Serial('/dev/ttyACM0')

def start_node():
    rospy.init_node('serial sender')
    rospy.subscriber('PID', data_type, callback)
    rospy.spin()

def callback(data):
    ser.write(data.encode())

if __name__ == '__main__':
    start_node()
