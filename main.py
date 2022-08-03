import YB_Pcb_Car
import ultrasonico as dis
import time

car = YB_Pcb_Car.YB_Pcb_Car()



while(True):

    distance = dis.Distance_test()

    if(distance > 50):
        car.Car_Run(250,250)

    elif( distance < 50 & distance > 5):
        vel = car.Car_Run(int(distance),int(distance))
    elif(distance < 5):
        car.Car_Stop()

