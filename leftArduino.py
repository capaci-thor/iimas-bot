#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Raspberry Pi to Arduino I2C Communication
#i2cdetect -y 1

#library
import sys
import smbus
import time

# Slave Addresses
I2C_SLAVE_ADDRESS = 0x8 #0x0b ou 11

# This function converts a string to an array of bytes.
def ConvertStringsToBytes(src):
  converted = []
  for b in src:
    converted.append(ord(b))
  return converted

def main(args):
    # Create the I2C bus
    bus = smbus.SMBus(1)
  
    slaveAddress = I2C_SLAVE_ADDRESS
 
    BytesToSend = ConvertStringsToBytes("1")
    print("Sent " + str(slaveAddress) + " the " + str("1") + " command.")
    print(BytesToSend )
    bus.write_byte_data(slaveAddress,  0x38, 1)
    time.sleep(1)

    while True:
        try:
            data=bus.read_byte_data(slaveAddress,0x00)
            print("recieve from slave:")
            print(data)
        except:
            print("remote i/o error")
            time.sleep(0.5)
    return 0

if __name__ == '__main__':
     try:
        main(sys.argv)
     except KeyboardInterrupt:
        print("program was stopped manually")
     input()