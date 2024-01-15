from serial import *
import smbus
import time
from math import *
import YB_Pcb_Car


#constants
#ticks of encoder
ticks = 20
##wheel diameter [m]
wheelD = 0.066 #[m]
#length between wheels [m]
b = 0.14 #[m]

#Arduino direction for serial com 
ser = Serial(port='/dev/ttyAMA0', baudrate = 9600, parity=PARITY_NONE,
              stopbits=STOPBITS_ONE, bytesize=EIGHTBITS, timeout=10)

#Arduino was configured for this adress
I2C_SLAVE_ADDRESS = 0x8

car = YB_Pcb_Car.YB_Pcb_Car()


#Function get encoders data
def encoders():
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
        time.sleep(0.5)
    vector.append(c_l)

    #send msg
    return vector

def ConvertStringsToBytes(src):

    converted = []
    for b in src:
        converted.append(ord(b))
    return converted 

def getVelocity(start_time, singL, singR):
    #Get data of encoders

    countEncoders = encoders()
    countRight = countEncoders[0]
    countLeft = countEncoders[1]

    # Tiempo 
    elapsed_time = time.time_ns() - start_time #[ns]
    elapsed_time = elapsed_time / 1000000000 # [s]

    #Convert to rpm
    rpmRight = (60 / elapsed_time) * (countRight / ticks)
    rpmLeft = (60 / elapsed_time) * (countLeft / ticks)

    #Convert to m/s
    VelRight = (rpmRight / 60) * pi * wheelD * singR
    velLeft = (rpmLeft / 60) * pi * wheelD * singL
    
    #see velocitys
    print("Velocidad der : " + str(VelRight))
    print("Velocidad izq : " + str(velLeft))

    #write Vel for document
    #file.write(str(VelRight)+ ',' + str(velLeft)+ ',')
    #return 
    return VelRight,velLeft,elapsed_time

qPIDR = 0.0 
q1PIDR = 0.0

Error0R = 0.0
Error1R = 0.0
Error2R = 0.0

qPIDL = 0.0 
q1PIDL = 0.0

Error0L = 0.0
Error1L = 0.0
Error2L = 0.0

kp = 3.0
ki = 7.0
kd = 0.01

vMeas = []
wMeas = []

while True:
    start_time = time.time_ns()

    spL = 0.5 #m/s
    spR = 1.5 #m/s

    velMeas = getVelocity(start_time, 1,1)

    VelRight = velMeas[0]
    velLeft = velMeas[1]
    elapsed_time = velMeas[2]

    Error0R = spR - VelRight

    #Diferential equation

    qPIDR = q1PIDR + (kp+kd/elapsed_time)*(Error0R) + (kp + ki*elapsed_time - 2*(kd/elapsed_time))*(Error1R) + (kd/elapsed_time)*(Error2R)
    q1PIDR = qPIDR

    Error2R = Error1R
    Error1R = Error0R

    qPIDL = q1PIDL + (kp+kd/elapsed_time)*(Error0L) + (kp + ki*elapsed_time - 2*(kd/elapsed_time))*(Error1L) + (kd/elapsed_time)*(Error2L)
    q1PIDL = qPIDL

    Error2L = Error1L
    Error1L = Error0L

    print("qPIDR_PRE: " + str(qPIDR))

    if(qPIDR > 500):
        qPID = 500
    elif(qPID < 63):
        qPID = 63

    print("qPIDR_POST: " + str(qPIDR))
    outR = int(qPIDR * (255/500))
    print("outR: " + str(outR))

    ##### LEFT
    print("qPIDL_PRE: " + str(qPIDL))
    if(qPIDL > 500):
        qPIDL = 500
    elif(qPIDL < 63):
        qPIDL = 63

    print("qPIDL_POST: " + str(qPIDR))
    outL = int(qPIDL * (255/500))
    print("outL: " + str(outL))

    car.Control_Car(outL , outR)


    #No me importa ahorita
    v = (VelRight + velLeft)/2
    w = (VelRight - velLeft)/b

    vMeas.append(velMeas[0])
    wMeas.append(velMeas[1])
    time.sleep(0.4)

