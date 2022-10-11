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

def main():
    # Create the I2C bus
    bus = smbus.SMBus(1)
  
    slaveAddress = I2C_SLAVE_ADDRESS
 
    BytesToSend = ConvertStringsToBytes("1")
    print("Sent " + str(slaveAddress) + " the " + str("1") + " command.")
    print(BytesToSend )
    bus.write_byte(slaveAddress,  1)
    time.sleep(1)

    while True:
        try:
            data=bus.read_byte(slaveAddress)
            print("recieve from slave:")
            print(data)
        except:
            print("remote i/o error")
            time.sleep(0.5)
    return 0

main()