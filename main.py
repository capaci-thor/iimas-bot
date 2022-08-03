import YB_Pcb_Car
import ultrasonico as dis
import time

car = YB_Pcb_Car.YB_Pcb_Car()

a = 0

while(a == 0):

    distance = dis.Distance_test()
    #a = input()
    if(a != 0):
        car.Car_Stop()

    if(distance > 50):
        car.Car_Run(100,100)

    elif( (distance < 50) and (distance > 5)):
        vel = car.Car_Run(int(distance)+30,int(distance)+30)
    elif(distance < 5):
        car.Car_Stop()




