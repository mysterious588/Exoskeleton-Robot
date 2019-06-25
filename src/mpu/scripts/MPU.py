#!/usr/bin/env python

import smbus
import math
import time
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float32
from mpu.msg import angles  # custom made message for storing values

GPIO.setmode(GPIO.BOARD)

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

# Scales gained from the datasheet
gyro_scale = 131.0
accel_scale = 16384.0

# MPU6050's address when pin AD0 is pulled to ground
address = 0x68  # only one MPU will have this address
rospy.init_node("MPU")
pub = rospy.Publisher("angles", angles, queue_size=10)  # publishes angles message
rate = rospy.Rate(200)  # four times the normal rate due to four MPUs

# Global variable for indicating the current MPU
count = 0

# The custom message which can be found in mpu/msg
angles = angles()

# The custom messages variables
angles.left_leg_angle = 0
angles.left_thigh_angle = 0
angles.right_leg_angle = 0
angles.right_thigh_angle = 0


# define four pins as output (AD0 pins are connected here)
def setup_pins(pins):
    for i in range(len(pins)):
        GPIO.setup(pins[i], GPIO.OUT)


def read_all():
    # Read 6 bytes from 0x43 to 0x48 (gyro out registers) & stores them in a list
    raw_gyro_data = bus.read_i2c_block_data(address, 0x43, 6)
    # Read 6 bytes from 0x3b to 0x40 (acce out registers) & stores them in a list
    raw_accel_data = bus.read_i2c_block_data(address, 0x3b, 6)
    # Data gathered from the MPU is the actual readings multiplied by a scale, so divide back
    gyro_scaled_x = twos_compliment((raw_gyro_data[0] << 8) + raw_gyro_data[1]) / gyro_scale
    gyro_scaled_y = twos_compliment((raw_gyro_data[2] << 8) + raw_gyro_data[3]) / gyro_scale
    gyro_scaled_z = twos_compliment((raw_gyro_data[4] << 8) + raw_gyro_data[5]) / gyro_scale
    # Same goes here
    accel_scaled_x = twos_compliment((raw_accel_data[0] << 8) + raw_accel_data[1]) / accel_scale
    accel_scaled_y = twos_compliment((raw_accel_data[2] << 8) + raw_accel_data[3]) / accel_scale
    accel_scaled_z = twos_compliment((raw_accel_data[4] << 8) + raw_accel_data[5]) / accel_scale

    return gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z


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


def getData():
    setup_pins([21, 22, 23, 24])
    K = 0.98 # Complementary filter gain
    K1 = 1 - K
    global last_x
    global last_y
    global last_z
    (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = read_all()
    last_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
    last_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
    last_z = get_z_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
    time_diff = 0.01
    global angles
    global count
    print("left leg \t left thigh \t right leg \t right thigh")
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

        if count == 0:  # left leg mpu
            angles.left_leg_angle = rotation_x
        elif count == 1:  # left thigh mpu
            angles.left_thigh_angle = rotation_x
        elif count == 2:  # t right leg mpu
            angles.right_leg_angle = rotation_x
        elif count == 3:  # right thigh mpu
            angles.right_thigh_angle = rotation_x
            pub.publish(angles)  # when count reaches four the angles will be ready to be published
            print(str(angles.left_leg_angle) + " " + str(angles.left_thigh_angle) + " " + str(
                angles.right_leg_angle) + " " + str(angles.right_thigh_angle))

        switchMPU([21, 22, 23, 24])

    # reset All pins in case the program is out
    GPIO.cleanup()


###################################
# switch AD0 Pins state and keep only one mpu at 0x68 --10k iq yes i know
###################################
def switchMPU(pins):
    global count
    if count == 0:
        for i in range(4):
            if count != i:
                GPIO.output(pins[i], True)  # pull up to ox69
            else:
                GPIO.output(pins[i], False)  # pull down to 0x68
    count = count + 1
    if count == 4:
        count = 0


if __name__ == "__main__":
    # Now wake the 6050 up as it starts in sleep mode
    bus.write_byte_data(address, power_mgmt_1, 0)
    getData()
    rate.sleep()
