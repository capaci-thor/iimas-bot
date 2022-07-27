import cv2

capture = cv2.VideoCapture(0)

while (capture.isOpened()):

    ret, frame = capture.read()
    ret,th1 = cv2.threshold(frame,127,255,cv2.THRESH_BINARY)


    cv2.imshow('webCam',th1)
    if (cv2.waitKey(1) == ord('s')):
        break

capture.release()
cv2.destroyAllWindows()