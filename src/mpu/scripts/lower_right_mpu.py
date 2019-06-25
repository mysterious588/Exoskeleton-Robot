#!/usr/bin/env python

import smbus
import math
import time
import rospy
from std_msgs.msg import Float32

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

# scales gained from the datasheet
gyro_scale = 131.0
accel_scale = 16384.0

# MPU6050's address when pin AD0 is pulled HIGH
address = 0x69
rospy.init_node("lower_right_mpu")
pub = rospy.Publisher("lower_right_angle", Float32, queue_size=10)
rate = rospy.Rate(50)


def read_all():
    # read 6 bytes from 0x43 to 0x48 (gyro out registers) & stores them in a list
    raw_gyro_data = bus.read_i2c_block_data(address, 0x43, 6)
    # read 6 bytes from 0x3b to 0x40 (acce out registers) & stores them in a list
    raw_accel_data = bus.read_i2c_block_data(address, 0x3b, 6)
    # data gathered from the MPU is the actual readings multiplied by a scale, so divide back
    gyro_scaled_x = twos_compliment((raw_gyro_data[0] << 8) + raw_gyro_data[1]) / gyro_scale
    gyro_scaled_y = twos_compliment((raw_gyro_data[2] << 8) + raw_gyro_data[3]) / gyro_scale
    gyro_scaled_z = twos_compliment((raw_gyro_data[4] << 8) + raw_gyro_data[5]) / gyro_scale
    # same goes here
    accel_scaled_x = twos_compliment((raw_accel_data[0] << 8) + raw_accel_data[1]) / accel_scale
    accel_scaled_y = twos_compliment((raw_accel_data[2] << 8) + raw_accel_data[3]) / accel_scale
    accel_scaled_z = twos_compliment((raw_accel_data[4] << 8) + raw_accel_data[5]) / accel_scale

    return gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z


def twos_compliment(val):
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val


def dist(a, b):
    return math.sqrt((a * a) + (b * b))


def get_y_rotation(x, y, z):
    radians = math.atan2(x, dist(y, z))
    return -math.degrees(radians)


def get_x_rotation(x, y, z):
    radians = math.atan2(y, dist(x, z))
    return math.degrees(radians)


def get_z_rotation(x, y, z):
    radians = math.atan2(z, dist(x, y))
    return math.degrees(radians)


bus = smbus.SMBus(1)  # or bus = smbus.SMBus(1) for Revision 2 boards


def getData():
    K = 0.98
    K1 = 1 - K
    global last_x
    global last_y
    global last_z
    (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = read_all()
    last_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
    last_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
    last_z = get_z_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
    time_diff = 0.01
    while not rospy.is_shutdown():
        (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = read_all()
        gyro_offset_x = gyro_scaled_x
        gyro_offset_y = gyro_scaled_y
        gyro_offset_z = gyro_scaled_z

        gyro_total_x = last_x - gyro_offset_x
        gyro_total_y = last_y - gyro_offset_y
        gyro_total_z = last_z - gyro_offset_z
        time.sleep(time_diff - 0.005)

        (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = read_all()

        gyro_scaled_x -= gyro_offset_x
        gyro_scaled_y -= gyro_offset_y
        gyro_scaled_z -= gyro_offset_z

        gyro_x_delta = (gyro_scaled_x * time_diff)
        gyro_y_delta = (gyro_scaled_y * time_diff)
        gyro_z_delta = (gyro_scaled_z * time_diff)

        gyro_total_x += gyro_x_delta
        gyro_total_y += gyro_y_delta
        gyro_total_z += gyro_z_delta

        rotation_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
        rotation_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
        rotation_z = get_z_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)

        last_x = K * (last_x + gyro_x_delta) + (K1 * rotation_x)
        last_y = K * (last_y + gyro_y_delta) + (K1 * rotation_y)
        last_z = K * (last_z + gyro_z_delta) + (K1 * rotation_z)

        pub.publish(rotation_x)
        rotation = str(rotation_x) + " " + str(rotation_y) + " " + str(rotation_z)
        print(rotation)


if __name__ == "__main__":
    # Now wake the 6050 up as it starts in sleep mode
    bus.write_byte_data(address, power_mgmt_1, 0)
    getData()
    rate.sleep()
