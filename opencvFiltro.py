import cv2

capture = cv2.VideoCapture(0)

while (capture.isOpened()):

    ret, frame = capture.read()
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    ret,th1 = cv2.threshold(gray_image,127,255,cv2.THRESH_BINARY)


    cv2.imshow('webCam',th1)
    if (cv2.waitKey(1) == ord('s')):
        break

capture.release()
cv2.destroyAllWindows()