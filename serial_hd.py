import time
from serial import *


ser = Serial(
    port='/dev/ttyAMA0',
    baudrate = 9600,
    parity=PARITY_NONE,
    stopbits=STOPBITS_ONE,
    bytesize=EIGHTBITS,
    timeout=5
    )

while 1:
    ser.write("0".encode())
    x=ser.readline().decode()
    x = x.replace("\n","")
    x.replace("\r","")
    print(x)
    time.sleep(1)