#!/usr/bin/env python

#############################
#right leg's AD0 is connected to VCC
#right thigh's is not connected
#############################


import smbus
import math
import time
import rospy
from std_msgs.msg import Float32

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

# Scales gained from the datasheet
gyro_scale = 131.0
accel_scale = 16384.0

# MPU6050's address when pin AD0 is pulled to ground
# only one MPU will have this address
rospy.init_node("MPU")
pub_right_leg = rospy.Publisher("right_leg_angle", Float32, queue_size=10)  # publishes angles message
pub_right_thigh = rospy.Publisher("right_thigh_angle", Float32, queue_size=10)  # publishes angles message
rate = rospy.Rate(50)  # four times the normal rate due to four MPUs

# Global variable for indicating the current MPU
count = 0


def twos_compliment(val):
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val


# some maths functions
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


bus = smbus.SMBus(1)  # I2C channel 1


class MPU:
    def __init__(self, address=0x68):
        self.address = address
        bus.write_byte_data(address, 0x6B, 0)

    def read_all(self):
        # Read 6 bytes from 0x43 to 0x48 (gyro out registers) & stores them in a list
        raw_gyro_data = bus.read_i2c_block_data(self.address, 0x43, 6)
        # Read 6 bytes from 0x3b to 0x40 (acce out registers) & stores them in a list
        raw_accel_data = bus.read_i2c_block_data(self.address, 0x3b, 6)
        # Data gathered from the MPU is the actual readings multiplied by a scale, so divide back
        gyro_scaled_x = twos_compliment((raw_gyro_data[0] << 8) + raw_gyro_data[1]) / gyro_scale
        gyro_scaled_y = twos_compliment((raw_gyro_data[2] << 8) + raw_gyro_data[3]) / gyro_scale
        gyro_scaled_z = twos_compliment((raw_gyro_data[4] << 8) + raw_gyro_data[5]) / gyro_scale
        # Same goes here
        accel_scaled_x = twos_compliment((raw_accel_data[0] << 8) + raw_accel_data[1]) / accel_scale
        accel_scaled_y = twos_compliment((raw_accel_data[2] << 8) + raw_accel_data[3]) / accel_scale
        accel_scaled_z = twos_compliment((raw_accel_data[4] << 8) + raw_accel_data[5]) / accel_scale

        return gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z


MPU_leg_right = MPU(address = 0x69)
MPU_thigh_right = MPU(address=0x68)


def get_leg_right_data():
    K = 0.98  # Complementary filter gain
    K1 = 1 - K
    global last_x
    global last_y
    global last_z
    (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y,
     accel_scaled_z) = MPU_leg_left.read_all()
    last_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
    last_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
    last_z = get_z_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
    time_diff = 0.01
    (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y,
     accel_scaled_z) = MPU_leg_right.read_all()
    gyro_offset_x = gyro_scaled_x
    gyro_offset_y = gyro_scaled_y
    gyro_offset_z = gyro_scaled_z

    gyro_total_x = last_x - gyro_offset_x
    gyro_total_y = last_y - gyro_offset_y
    gyro_total_z = last_z - gyro_offset_z

    time.sleep(time_diff - 0.005)

    (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y,
     accel_scaled_z) = MPU_leg_right.read_all()

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
    print("right leg", rotation_x)
    pub_right_leg.publish(rotation_x)


def get_thigh_right_data():
    K = 0.98  # Complementary filter gain
    K1 = 1 - K
    global last_x
    global last_y
    global last_z
    (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y,
     accel_scaled_z) = MPU_thigh_left.read_all()
    last_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
    last_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
    last_z = get_z_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
    time_diff = 0.01
    (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y,
     accel_scaled_z) = MPU_thigh_right.read_all()
    gyro_offset_x = gyro_scaled_x
    gyro_offset_y = gyro_scaled_y
    gyro_offset_z = gyro_scaled_z

    gyro_total_x = last_x - gyro_offset_x
    gyro_total_y = last_y - gyro_offset_y
    gyro_total_z = last_z - gyro_offset_z
    time.sleep(time_diff - 0.005)

    (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y,
     accel_scaled_z) = MPU_thigh_right.read_all()

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
    print("left thigh", rotation_x)
    pub_right_thigh.publish(rotation_x)


def getData():
    while not rospy.is_shutdown():
        get_leg_right_data()
        get_thigh_right_data()
        rate.sleep()


if __name__ == "__main__":
    getData()
    rate.sleep()
