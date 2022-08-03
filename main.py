import YB_Pcb_Car
import ultrasonico as dis
import time
import cv2

car = YB_Pcb_Car.YB_Pcb_Car()
capture = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc(*'avc1')
out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (640, 480))


while(cv2.waitKey(1) != ord('s')):
    ret, frame = capture.read()
    distance = dis.Distance_test()
    #a = input()
    if ret:
        out.write(frame)
        cv2.imshow('Video', frame)
        

    if(distance > 50):
        car.Car_Run(100,100)

    elif( (distance < 50) and (distance > 5)):
        vel = car.Car_Run(int(distance)+30,int(distance)+30)
    elif(distance < 5):
        car.Car_Stop()
        break


car.Car_Stop()
capture.release()
out.release()
cv2.destroyAllWindows()

