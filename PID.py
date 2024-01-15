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

qPID = 0.0 
q1PID = 0.0

Error0 = 0.0
Error1 = 0.0
Error2 = 0.0

kp = 1.0
ki = 5.0
kd = 0.01

vMeas = []
wMeas = []

while True:
    start_time = time.time_ns()

    spL = 0.5 #m/s
    spR = 1.0 #m/s

    velMeas = getVelocity(start_time, 1,1)

    VelRight = velMeas[0]
    velLeft = velMeas[1]
    elapsed_time = velMeas[2]

    Error0 = spR - VelRight
    #Diferential equation

    qPID = q1PID + (kp+kd/elapsed_time)*(Error0) + (kp + ki*elapsed_time - 2*(kd/elapsed_time))*(Error1) + (kd/elapsed_time)*(Error2)
    q1PID = qPID

    Error2 = Error1
    Error1 = Error0

    print("qPID_PRE: " + str(qPID))
    if(qPID > 500):
        qPID = 500
    elif(qPID < 60):
        q1PID = 60

    print("qPID_POST: " + str(qPID))
    outR = int(qPID * (255/500))
    
    print("outR: " + str(outR))

    car.Control_Car(0 , outR)


    #No me importa ahorita
    v = (VelRight + velLeft)/2
    w = (VelRight - velLeft)/b

    vMeas.append(velMeas[0])
    wMeas.append(velMeas[1])
    time.sleep(0.4)

