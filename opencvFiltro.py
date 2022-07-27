import cv2

capture = cv2.VideoCapture(0)

while (capture.isOpened()):

    ret, frame = capture.read()
    frame = cv2.medianBlur(frame, 5)
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    ret,th1 = cv2.adaptiveThreshold(gray_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)


    cv2.imshow('webCam',th1)
    if (cv2.waitKey(1) == ord('s')):
        break

capture.release()
cv2.destroyAllWindows()