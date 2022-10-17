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
    while True:
        x=ser.read().decode()
        if x > 0 :
            break
        #x = x.replace("\n","")
        x = int(x)
    print(x)
    time.sleep(1)