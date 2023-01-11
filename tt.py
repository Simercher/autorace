import cv2
import time
l = 71
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)
while 1:
    ret , frame = cap.read()
    cv2.imwrite(str(l)+".jpg",frame)
    time.sleep(1)
    if l > 80:
       break
    l+=1