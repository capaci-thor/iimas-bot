import cv2
import YB_Pcb_Car
import time

capture = cv2.VideoCapture(0)
car = YB_Pcb_Car.YB_Pcb_Car()

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'avc1')
out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (640, 480))

while (capture.isOpened()):
    ret, frame = capture.read()
    if ret:
        out.write(frame)
        cv2.imshow('Video', frame)
    car.Car_Run(50,50)
    if (cv2.waitKey(1) == ord('s')):
        car.Car_Stop()
        break

capture.release()
out.release()
cv2.destroyAllWindows()