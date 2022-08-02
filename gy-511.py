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
	#write to sample rate register for acc
	#bus.write_byte_data(ACCELEROMETER_ADDR, 0x20, 0x47)
	#bus.write_byte_data(ACCELEROMETER_ADDR, 0x23, 0x48)
    #write to sample rate register for mag
	bus.write_byte_data(MAG_ADDR, 0x00, 0x14)
	bus.write_byte_data(MAG_ADDR, 0x02, 0x00)
	

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(MAG_ADDR, addr)
        low = bus.read_byte_data(MAG_ADDR, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
ACCELEROMETER_ADDR = 0x19   # ACCELEROMETER  device address
MAG_ADDR = 0x1e   # MAG  device address

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
		acc_x = read_raw_data(0x29)
		acc_y = read_raw_data(0x2b)
		acc_z = read_raw_data(0x2d)
		
		#Accelerometer conver from g to m/s2
		Ax = acc_x
		Ay = acc_y
		Az = acc_z

		listAccX.append(Ax)
		listAccY.append(Ay)
		listAccZ.append(Az)
		
		
	#Sort all values
	listAccX.sort()
	listAccY.sort()
	listAccZ.sort()


	ax = 0
	ay = 0
	az = 0


	#Average of every variable
	for i in range(1,9):
		ax += listAccX[i]
		ay += listAccY[i]
		az += listAccZ[i]


	ax = round( ax/9, 2)
	ay = round( ay/9, 2)
	az = round( az/9, 2)



	print("Ax: " + str(ax) + " Ay: "+ str(ay) + " Az: "+ str(az))
	
	sleep(1)