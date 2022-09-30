import YB_Pcb_Car
import time

car = YB_Pcb_Car.YB_Pcb_Car()

while(True):
    car.Car_Run(0,50)
    time.sleep(1)
    car.Car_Run(0,0)
