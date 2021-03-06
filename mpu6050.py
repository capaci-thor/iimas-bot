# -*- coding: utf-8 -*-

import smbus
from time import sleep


#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38

ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F

GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()


while True:

	listAccX = []
	listAccY = []
	listAccZ = []

	listGyroX = []
	listGyroY = []
	listGyroZ = []

	
	for i in range(0,10):
		#Read Accelerometer raw value and convert to g units
		acc_x = read_raw_data(ACCEL_XOUT_H)/16384.0
		acc_y = read_raw_data(ACCEL_YOUT_H)/16384.0
		acc_z = read_raw_data(ACCEL_ZOUT_H)/16384.0
		
		#Read Gyroscope raw value
		#and scale range +/- 250 degree/C as per sensitivity scale factor
		gyro_x = read_raw_data(GYRO_XOUT_H)/131.0
		gyro_y = read_raw_data(GYRO_YOUT_H)/131.0
		gyro_z = read_raw_data(GYRO_ZOUT_H)/131.0
		
		#Accelerometer conver from g to m/s2
		Ax = acc_x*9.81
		Ay = acc_y*9.81
		Az = acc_z*9.81

		listAccX.append(Ax)
		listAccY.append(Ay)
		listAccZ.append(Az)
		
		Gx = gyro_x
		Gy = gyro_y
		Gz = gyro_z
		listGyroX.append(Gx)
		listGyroY.append(Gy)
		listGyroZ.append(Gz)
		
	#Sort all values
	listAccX.sort()
	listAccY.sort()
	listAccZ.sort()

	listGyroX.sort()
	listGyroY.sort()
	listGyroZ.sort()


	ax = 0
	ay = 0
	az = 0
	gx = 0
	gy = 0
	gz = 0

	#Average of every variable
	for i in range(1,9):
		ax += listAccX[i]
		ay += listAccY[i]
		az += listAccZ[i]

		gx += listGyroX[i]
		gy += listGyroY[i]
		gz += listGyroZ[i]

	ax = round( ax/9, 2)
	ay = round( ay/9, 2)
	az = round( az/9, 2)


	gx = round( gx/9, 2)
	gy = round( gy/9, 2)
	gz = round( gz/9, 2)


	print("Ax: " + str(ax) + " Ay: "+ str(ay) + " Az: "+ str(az))
	print("Gx: " + str(gx) + " Gy: "+ str(gy) + " Gz: "+ str(gz))
	sleep(1)