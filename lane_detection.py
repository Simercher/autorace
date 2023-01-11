import cv2
import numpy as np
import utilis
from adafruit_servokit import ServoKit
import busio

from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

curveList = []
xlist = []
ylist = []
avgVal = 10

def getLaneCurve(frame):
     # initialTrackerBarVals = [55, 32, 0, 106]
     initialHSVValue = [0, 180, 0, 255, 0, 181, 0, 0]
     angle = 80
     lv = 0.6
     rv = -0.6
     imgCopy = frame.copy()
     # imgcrop = frame.copy()
     # kernal = np.ones((3, 3), np.uint8)

     # imgcrop = utilis.contrastandbrightness(img, initialHSVValue)
     # imgcrop = imgcrop[120: , : , : ]

     # imgThresYellow = utilis.thresholding(imgcrop, initialHSVValue[:8])
     # imgThresWhite = utilis.thresholding(imgcrop, initialHSVValue[8:])
     mask = utilis.thresholding(frame, initialHSVValue)
     
     h, w, c= frame.shape
     points = utilis.ValTrackers()

     imgwarpPoints = utilis.drawpoint(imgCopy, points)

     # imgwarpYellow = utilis.warpImg(imgThresYellow, points, w, h)
     # imgwarpWhite = utilis.warpImg(imgThresWhite, points, w, h)
     imgwarp = utilis.warpImg(mask, points, w, h)

     '''
     areaY = utilis.getArea(imgwarpYellow)
     areaW = utilis.getArea(imgwarpWhite)
     
     p = areaY / areaW
     if p >= 2:
          p = 2
     if areaW == 0 and areaY == 0:
          angle = 80
     elif areaW == 0:
          angle += int(100 * 0.6)
     elif areaY == 0:
          angle -= int(100 * 0.6)
     elif areaW > areaY:
          # print("turn left")
          angle -= int((100 - p*100) * 0.6) 
     elif areaW < areaY:
          # print("turn right")
          angle -= int((100 - p*100) * 0.6)
     print(angle)
     '''
     # laneThres = cv2.bitwise_xor(imgThresYellow, imgThresWhite)
     # imgwarp2 = utilis.warpImg(imgThres2, points, w, h)
     # imgwarpPoints = utilis.drawpoint(imgCopy, points)

     middlePoint, imgHist = utilis.getHistogram(imgwarp, display = True, minPer = 0.1, region=4)
     # # # print(middlePoint1)    
     curveAvergaePoint, imgHist = utilis.getHistogram(imgwarp, display = True, minPer = 0.9)
     curveRaw = curveAvergaePoint - middlePoint
     # print(curveRaw)
     curveList.append(curveRaw)
     # print(curveList)
     if len(curveList) > avgVal:
          curveList.pop(0)
     curve = sum(curveList)/len(curveList)
     # print(curve)
     if curve < -60:
          curve = -60
     elif curve > 60:
          curve = 60
     
     angle += curve
     print(angle)

     # cv2.imshow("Line", imgCopy)
     # cv2.imshow("WarpY", imgwarpYellow)
     cv2.imshow("Warp", imgwarp)
     cv2.imshow('Points', imgwarpPoints)
     # cv2.imshow("imgHist", imgHist)
     # cv2.imshow("ThresholdW", imgThresWhite)
     # cv2.imshow("ThresholdY", imgThresYellow)
     # cv2.imshow('mask', mask)
     if angle <40:
          # rv = -1
          # lv= 0.4
          rv, lv = utilis.getV()
          angle = 30

     return angle, lv, rv

def main():
     i2c = busio.I2C(SCL, SDA)

     kit = ServoKit(channels = 16)

     pca = PCA9685(i2c, address=0x40)
     pca.frequency = 100

     pca.channels[15].duty_cycle = 0xFFFF
     pca.channels[9].duty_cycle = 0xFFFF

     motor1 = motor.DCMotor(pca.channels[14], pca.channels[13])
     motor2 = motor.DCMotor(pca.channels[10], pca.channels[11])
     motor1.throttle = 0.3
     motor2.throttle = -0.3
     # print("test")
     cap = cv2.VideoCapture(0)
     # print("test")
     initialTrackerBarVals = [30, 180, 0, 240]
     # initialHSVValue = [17, 67, 115,81, 255, 255, 0, 0, 30, 0, 221, 180, 39, 255, 0, 22]
     cap.set(3, 640)
     cap.set(4, 480)
     # constrast = 200
     # brightness = 0
     utilis.initializeTrackerBar(initialTrackerBarVals)
     
     while True:
          try:
               ret, frame = cap.read()
               frame = cv2.resize(frame, (480, 240))
               # cv2.imshow("LineDetection", frame) 
               angle, lv, rv = getLaneCurve(frame)
               if cv2.waitKey(1) == ord('q'):
                    motor1.throttle = 0
                    motor2.throttle = 0

                    pca.deinit()
                    break
               motor1.throttle = lv
               motor2.throttle = rv
               kit.servo[7].angle = angle
          except KeyboardInterrupt:
               motor1.throttle = 0
               motor2.throttle = 0

               pca.deinit()
               break
               # cv2.destroyALLWindow()

if __name__ == '__main__':
     kit = ServoKit(channels = 16)

     kit.servo[7].angle = 80
     main()