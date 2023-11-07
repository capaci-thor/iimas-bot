import YB_Pcb_Car
from serial import *
import smbus
import time
from math import *
import pandas as pa
from scipy import stats

#constants
#ticks of encoder
ticks = 20
##wheel diameter [m]
wheelD = 0.066 #[m]
#length between wheels [m]
b = 0.1 #[m]

#Arduino direction for serial com 
ser = Serial(port='/dev/ttyAMA0', baudrate = 9600, parity=PARITY_NONE,
              stopbits=STOPBITS_ONE, bytesize=EIGHTBITS, timeout=10)

#Arduino was configured for this adress
I2C_SLAVE_ADDRESS = 0x8


# Get data for Linear regression of motors
path ='/home/rotjeot/'
pd = pa.read_csv(path + "data.csv")
pwm = pd["pwm"]
vel_r = pd["v_r"]
vel_l = pd["v_l"]
slope_r, intercept_r, r_r, p_r, std_err_r = stats.linregress(vel_r, pwm)
slope_l, intercept_l, r_l, p_l, std_err_l = stats.linregress(vel_l, pwm)

#Create object for send vel to motors
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

def getVelocity(elapsed_time):
    #Get data of encoders

    countEncoders = encoders()
    countRight = countEncoders[0]
    countLeft = countEncoders[1]

    #Convert to rpm
    rpmRight = (60 / elapsed_time) * (countRight / ticks)
    rpmLeft = (60 / elapsed_time) * (countLeft / ticks)
    print("rpm R" + str(rpmRight))
    print("rpm L" + str(rpmLeft))
    #Convert to m/s
    VelRight = (rpmRight / 60) * pi * wheelD
    velLeft = (rpmLeft / 60) * pi * wheelD
    v = (VelRight + velLeft)/2
    w = (VelRight - velLeft)/b

    return v,w
    


def lyapunov():
    ## coordinates
    #initial coordinates[m]
    xIni = 0.0 #[m]
    yIni = 0.0 #[m]
    phiIni = 0.0 * (pi/180) #[rad]

    #goal coordinates [m]
    xGoal = 1.0 #[m]
    yGoal = 1.0 #[m]
    phiGoal = 0.0 * (pi/180) #[rad]

    ### List
    # Creation of list for lyapunov 
    ## Positions
    xPos = []
    yPos = []
    phiPos = []

    ## Velocitys
    # Calculated
    vCal = []
    wCal = []
    # Measured
    vMeas = []
    wMeas = []

    ## Errors 
    l = []
    rho = []
    theta = []

    #Initial list 
    xPos.append(xIni)
    yPos.append(yIni)
    phiPos.append(phiIni)

    #Aux var for iteration
    i = 0
    while True:
        start_time = time.time_ns()

        #Errors
        l.append( sqrt((xGoal - xPos[i])**2 + (yGoal - yPos[i])**2) )

        rho.append( atan2(yGoal - yPos[i], xGoal - xPos[i]) - phiPos[i])

        theta.append( atan2(yGoal - yPos[i], xGoal - xPos[i]) - phiGoal[i])

        #control parameters
        k1 = 0.2
        k2 = 0.2

        # Control

        vCal.append( k1 * cos(rho[i]) * l[i] )

        wCal.append( k2 * rho[i] + (k1/rho[i]) * cos(rho[i]) * sin(rho[i]) * (rho[i] + theta[i]) )

        # Send vel to robot
        robot(vCal, wCal)
        
        # Tiempo 
        elapsed_time = time.time_ns() - start_time #[ns]
        elapsed_time = elapsed_time / 1000000000 # [s]

        velMeas = getVelocity(elapsed_time)
        vMeas.append(velMeas[0])
        wMeas.append(velMeas[1])

        # integral
        phiPos.append(phiPos[i] + elapsed_time * wMeas[i])

        # Modelo cinemático

        xtmp = vMeas[i] * cos(phiPos[i+1])
        ytmp = wMeas[k] * sin(phiPos[i+1])

        # integral 
        xPos.append = xPos[i] + elapsed_time * xtmp
        yPos.append = xPos[i] + elapsed_time * ytmp
        i = i + 1




def robot(v,w):
    print(v,w)



velInput = float(input("Ingresa la velocidad lineal: "))
wInput = float(input("Ingresa la velocidad angular: "))

for i in range(30):
    r = wheelD/2
    wr = velInput + (b*wInput)/2 #(velInput + (b*wInput))/r
    wl = velInput - (b*wInput)/2#(velInput - (b*wInput))/r

    outL = int( (slope_l * wl) + intercept_l )
    outR = int( (slope_r * wr) + intercept_r )
    car.Control_Car(outL , outR)
    start_time = time.time_ns()
    vCalculada = (wr+wl) /2
    wCalculada = (wr-wl) /b

    time.sleep(0.15)
    # Tiempo 
    elapsed_time = time.time_ns() - start_time #[ns]
    elapsed_time = elapsed_time / 1000000000 # [s]

    velMeas = getVelocity(elapsed_time)
    print("Velocidad Lineal Calculada: " + str(vCalculada))
    print("Velocidad Angular Calculada: " + str(wCalculada))
    print("Velocidad Real: " + str(velMeas[1]) +"/" + str(outL) + "," + str(outR))
    print(elapsed_time)

car.Control_Car(0 , 0)