import cv2
import YB_Pcb_Car
import time

capture = cv2.VideoCapture(0)
car = YB_Pcb_Car.YB_Pcb_Car()

while (capture.isOpened()):
    ret, frame = capture.read()
    cv2.imshow('webCam',frame)
    car.Car_Run(150,150)
    if (cv2.waitKey(1) == ord('s')):
        car.Car_Stop()
        break

capture.release()
cv2.destroyAllWindows()