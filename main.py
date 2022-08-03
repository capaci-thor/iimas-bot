import YB_Pcb_Car
import ultrasonico as dis
import time

car = YB_Pcb_Car.YB_Pcb_Car()



while(True):

    try:
        distance = dis.Distance_test()
        time.sleep(1)
    except KeyboardInterrupt:
        pass

    if(distance > 50):
        car.Car_Run(250,250)

    elif( distance < 50 & distance > 5):
        vel = car.Car_Run(distance,distance)
    elif(distance < 5):
        car.Car_Stop()

