import YB_Pcb_Car
from serial import *
import smbus
import time
from math import *
import pandas as pa
from scipy import stats
from datetime import datetime

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

# Save data in csv file
timestamp = time.time()
date_time = datetime.fromtimestamp(timestamp)
str_date_time = date_time.strftime("%d-%m-%Y_%H-%M-%S")
pathDatos = '/home/rotjeot/datos/'
file = open(pathDatos + 'datos_' + str_date_time+'.csv', 'x')
file.write('i,vel_rigth,vel_left,v,w,vr,wr,l,rho, tetha, x,y,phi, xR, YR\n')

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

##Auxiliar variables for modify vel
auxWr = 0.0
auxWl = 0.0

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
    v = (VelRight + velLeft)/2
    w = (VelRight - velLeft)/b

    #write Vel for document
    file.write(str(VelRight)+ ',' + str(velLeft)+ ',')
    #return 
    return v,w,elapsed_time
    


def lyapunov():
    #global variables
    global auxWr
    global auxWl

    ## coordinates
    #initial coordinates[m]
    xIni = 0.0 #[m]
    yIni = 0.0 #[m]
    phiIni = 0.0 * (pi/180) #[rad]

    #goal coordinates [m]
    xGoal = 0.5 #[m]
    yGoal = 0.0 #[m]
    phiGoal = 90.0 * (pi/180) #[rad]

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
        file.write(str(i)+',')
        print(i)
        #Errors
        l.append( sqrt((xGoal - xPos[i])**2 + (yGoal - yPos[i])**2) )

        rho.append( atan2(yGoal - yPos[i], xGoal - xPos[i]) - phiPos[i])

        theta.append( atan2(yGoal - yPos[i], xGoal - xPos[i]) - phiGoal)

        #control parameters
        k1 = 0.09
        k2 = 0.09

        # Control

        vCal.append( k1 * cos(rho[i]) * l[i] )
        try:
            wCal.append( k2 * rho[i] + (k1/rho[i]) * cos(rho[i]) * sin(rho[i]) * (rho[i] + theta[i]) )
        except:
            wCal.append(0)

        # Output for every motor
        wr = vCal[i] + ((b*wCal[i])/2) + auxWr
        wl = vCal[i] - ((b*wCal[i])/2) + auxWl

        outL = int( (slope_l * wl) + intercept_l )
        outR = int( (slope_r * wr) + intercept_r )

        # Send vel to robot
        car.Control_Car(outL , outR)
        time.sleep(0.5)

        try:
            auxOutL = outL/abs(outL)
        except:
            auxOutL = 0

        try:
            auxOutR = outR/abs(outR)
        except:
            auxOutR = 0
        #Get real velocity
        velMeas = getVelocity(start_time, auxOutL, auxOutR)
        vMeas.append(velMeas[0])
        wMeas.append(velMeas[1])
        elapsed_time = velMeas[2]

        #Fix vel 
        if ((velMeas[0] == 0 and velMeas[1] == 0) and (vCal[i] != 0 or wCal[i] != 0)):
            auxWr = wr + auxWr #+ 2
            auxWl = wl + auxWl #+ 2
            print("Aux" + str(auxWl) + "," + str(auxWr))
        else:
            auxWr = 0
            auxWl = 0

        
        print("Velocidad Lineal Calculada : " + str(vCal[i]))
        print("Velocidad angular Calculada: " + str(wCal[i]*(180/pi)))

        print("Velocidad Lineal Real : " + str(velMeas[0]))
        print("Velocidad angular Real: " + str(velMeas[1]*(180/pi)))

        print("Error l     : " + str(l[i]))
        print("Error rho   : " + str(rho[i]*(180/pi)))
        print("Error theta : " + str(theta[i]))#*(180/pi))) 
        
        # integral
        phiPos.append( phiPos[i] + (elapsed_time * wMeas[i]) )

        # Modelo cinemático

        xtmp = elapsed_time * vMeas[i] * cos(phiPos[i+1])
        ytmp = elapsed_time * vMeas[i] * sin(phiPos[i+1])

        print("posicion x: " + str(xtmp))
        print("posicion y: " + str(ytmp))

        # integral 
        xPos.append( xPos[i] + (xtmp) )
        yPos.append( yPos[i] + (ytmp) )

        print("phi       : " + str(phiPos[i+1]*(180/pi)))
        print("posicion acumulada x: " + str(xPos[i+1]))
        print("posicion acumulada y: " + str(yPos[i+1]))

        file.write(str(vCal[i])+',' + str(wCal[i]) + ',' + str(velMeas[0]) + ',' + str(velMeas[1]) + ',' +
                    str(l[i]) + ',' + str(rho[i]) + ',' + str(theta[i]) + ',' + str(xtmp) + ',' + str(ytmp) +
                    ',' + str(phiPos[i+1]) + ',' +  str(xPos[i+1]) + ',' +  str(yPos[i+1]) + '\n')
    
        if(l[i] <= 0.1 and rho[i] <= 0.01 and theta[i] > -0.01 and theta[i] < 0.01):
            car.Control_Car(0 , 0)
            file.close()
            exit()
    
        i = i + 1
        #if(i == 4):
        #    car.Control_Car(0 , 0)
        #    exit()





# velInput = float(input("Ingresa la velocidad lineal: "))
# wInput = float(input("Ingresa la velocidad angular: "))

# for i in range(7):
#     #r = wheelD/2
#     wr = velInput + ((b*wInput)/2) #(velInput + (b*wInput))/r
#     wl = velInput - ((b*wInput)/2) #(velInput - (b*wInput))/r
    
#     #print("wr: "+ str(wr) + "wl: " + str(wl))

#     outL = int( (slope_l * wl) + intercept_l )
#     outR = int( (slope_r * wr) + intercept_r )
#     car.Control_Car(outL , outR)
#     start_time = time.time_ns()
#     vCalculada = (wr+wl) /2
#     wCalculada = (wr-wl) /b

#     time.sleep(0.2)
#     # Tiempo 
#     elapsed_time = time.time_ns() - start_time #[ns]
#     elapsed_time = elapsed_time / 1000000000 # [s]

#     velMeas = getVelocity(elapsed_time, outL/abs(outL), outR/abs(outR))
#     #print("Velocidad Lineal Calculada: " + str(vCalculada))
#     print("Velocidad Angular Calculada: " + str(wCalculada))
#     #print("Velocidad Lineal Real: " + str(velMeas[0]))# +"/" + str(outL) + "," + str(outR))
#     print("Velocidad Angular Real: " + str(velMeas[1]))
#     #print(elapsed_time)
try:
    lyapunov()
except KeyboardInterrupt:
    car.Control_Car(0 , 0)