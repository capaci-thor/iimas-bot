import time
import serial


ser = Serial(
    port='/dev/ttyS0',
    baudrate = 9600,
    parity=0,
    stopbits=1,
    bytesize=8,
    timeout=1
)

while 1:
    ser.write(b'A')
    x=ser.readline()
    print (x)
    time.sleep(1)