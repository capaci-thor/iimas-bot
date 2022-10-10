import time
from serial import *


ser = Serial(
    port='/dev/ttyS0',
    baudrate = 9600,
    parity=PARITY_NONE,
    stopbits=1,
    bytesize=8,
    timeout=1
)

while 1:
    x=ser.readline()
    print (x)
    #time.sleep(1)