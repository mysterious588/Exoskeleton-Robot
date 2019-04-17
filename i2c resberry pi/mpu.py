import RPI.GPIO as gpio #import GPIO package to control GPIO on a Rasberry pi
import smbus  #import SMBus module of I2C
from time import sleep #import this module to define sleep == delay

#MPU Registers and their Address
PWR_M = 0x6B
DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG  = 0x1B
INT_EN   = 0x38
ACCEL_X = 0x3B
ACCEL_Y = 0x3D
ACCEL_Z = 0x3F
GYRO_X  = 0x43
GYRO_Y  = 0x45
GYRO_Z  = 0x47

bus = smbus.SMBus(1)

device_address = 0x68 #MPU Adress

"""""""""""""""""""""
" @breif      : initializing the mpu
" @parameters : this function doesn't take any parameters
" @return     : void function
"""""""""""""""""""""
def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(device_address, DIV, 7)
    #write to power management register
    bus.write_byte_data(device_address, PWR_M, 1)
    #write to Configuration register
    bus.write_byte_data(device_address, CONFIG, 0)
    #write to Gyro configuration register
    bus.write_byte_data(device_address, GYRO_CONFIG, 24)
    #write to interrupt enable register
    bus.write_byte_data(device_address, INT_EN, 1)
    time.sleep(1)

def MPU_data_read(address):
    #Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(device_address, address)
    low = bus.read_byte_data(device_address, address+1)

     #concatenate higher and lower value
    value = ((high << 8) | low)

    #to get signed value from mpu6050
    if value > 32768:
        value = value - 65536

    return value


"""""""""""""""""""""
" @breif      : reading the accelerometer value
" @parameters : this function doesn't take any parameters
" @return     : acceleration in x, y, Z
"""""""""""""""""""""
def accel():
    x = MPU_data_read(ACCEL_X)
    y = MPU_data_read(ACCEL_Y)
    z = MPU_data_read(ACCEL_Z)

    Ax = x/16384.0
    Ay = y/16384.0
    Az = z/16384.0

    print(Ax, " - ", Ay, " - ", Az)
    time.sleep(.01)

"""""""""""""""""""""
" @breif      : reading the Gyroscope value
" @parameters : this function doesn't take any parameters
" @return     : degree per second in x, y, Z
"""""""""""""""""""""
def gyro():
    x = MPU_data_read(GYRO_X)
    y = MPU_data_read(GYRO_Y)
    z = MPU_data_read(GYRO_Z)

    Gx = x/131.0
    Gy = y/131.0
    Gz = z/131.0

    print(Gx, " - ", Gy, " - ", Gz)
    time.sleep(.01)

MPU_Init()

print("Reading data from MPU")

while True:
    accel()
    gyro()
    sleep(1)
