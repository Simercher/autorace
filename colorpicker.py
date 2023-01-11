import cv2
import numpy as np

def passFunction(x):
     pass

def colorPicker():
     windowName = 'ColorPicker'
     cv2.namedWindow(windowName)

     cv2.createTrackbar('HUE Min', windowName, 0, 180, passFunction)
     cv2.createTrackbar('HUE Max', windowName, 180, 180, passFunction)
     cv2.createTrackbar('SAT Min', windowName, 0, 255, passFunction)
     cv2.createTrackbar('SAT Max', windowName, 255, 255, passFunction)
     cv2.createTrackbar('VALUE Min', windowName, 0, 255, passFunction)
     cv2.createTrackbar('VALUE Max', windowName, 0, 255, passFunction)
     cv2.createTrackbar('Contrast', windowName, 0, 100, passFunction)
     cv2.createTrackbar('Brightness', windowName, 0, 100, passFunction)

def main():
     colorPicker()
     cam = cv2.VideoCapture(0)
     cam.set(3, 640)
     cam.set(4, 480)
     # fps = cam.get(cv2.CAP_PROP_FPS)
     while True:
          _, imgHSV = cam.read()
          cv2.imshow('picture', imgHSV)
          contrast = cv2.getTrackbarPos('Contrast', "ColorPicker")
          brightness = cv2.getTrackbarPos('Brightness', "ColorPicker")
          imgHSV = imgHSV * (contrast/127 + 1) - contrast + brightness
          imgHSV = np.clip(imgHSV, 0, 255)
          imgHSV = np.uint8(imgHSV)
          # cv2.imshow("1", imgHSV)
          imgHSV = cv2.cvtColor(imgHSV, cv2.COLOR_BGR2HSV)
          h_min = cv2.getTrackbarPos('HUE Min', "ColorPicker")
          h_max = cv2.getTrackbarPos('HUE Max', "ColorPicker")
          s_min = cv2.getTrackbarPos('SAT Min', "ColorPicker")
          s_max = cv2.getTrackbarPos('SAT Max', "ColorPicker")
          v_min = cv2.getTrackbarPos('VALUE Min', "ColorPicker")
          v_max = cv2.getTrackbarPos('VALUE Max', "ColorPicker")
          lowerWhite = np.array([h_min, s_min, v_min])
          upperWhite = np.array([h_max, s_max, v_max])
          maskWhite = cv2.inRange(imgHSV, lowerWhite, upperWhite)
          cv2.imshow("", maskWhite)
          if cv2.waitKey(10) == ord('q'):
               break


if __name__ == '__main__':
     main()