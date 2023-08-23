from serial import *
import smbus
from time import sleep
import math
import YB_Pcb_Car

car = YB_Pcb_Car.YB_Pcb_Car()   

ser = Serial(
        port='/dev/ttyAMA0',
        baudrate = 9600,
        parity=PARITY_NONE,
        stopbits=STOPBITS_ONE,
        bytesize=EIGHTBITS,
        timeout=10
        )

I2C_SLAVE_ADDRESS = 0x8 #Arduino was configured for this adress

path = '/home/rotjeot/'
file = open(path + 'read.csv', 'w')


def get_msg():
    diametro = 6.6
    global ser
    vector = []


    #code for right encoder
    ser.write("0".encode())
    c_r = ser.readline().decode()
    c_r = c_r.replace("\n","")
    c_r = c_r.replace("\r","")
    vector.append(int(c_r))


    #code for left encoder
    bus = smbus.SMBus(1)
    slaveAddress = I2C_SLAVE_ADDRESS

    BytesToSend = ConvertStringsToBytes("1")
    bus.write_byte(slaveAddress,  1)
    try:
        bus.write_byte(slaveAddress,  1)
        c_l = bus.read_byte(slaveAddress)
        c_l = int(c_l)            
    except:
        print("remote i/o error")
        sleep(0.5)
    vector.append(c_l)
    print(vector)
    rpm_r = (vector[0]/20)*60
    rpm_l = (vector[1]/20)*60
    vel_r = (math.pi*diametro*rpm_r)/(100*60)
    vel_l = (math.pi*diametro*rpm_l)/(100*60)
    return str(rpm_r)+ ','+ str(vel_r) + ','+ str(rpm_l)+ ','+ str(vel_l)+'\n'


def ConvertStringsToBytes(src):
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted   

a = get_msg()
file.write('pwm,rpm_r,v_r,rpm_l,v_l\n')
for i in range(0,256,5):
    print(i)
    car.Car_Run(i , i)
    sleep(1)
    msg = get_msg()
    to_save = str(i)+',' + msg
    file.write(to_save)

car.Car_Run(0 , 0)
file.close()