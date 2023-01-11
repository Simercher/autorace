# from curses import window
import cv2
import numpy as np

def contrastandbrightness(img, initialHSV):
     contrast = initialHSV[0]
     brightness = initialHSV[1]
     # contrast = cv2.getTrackbarPos('Contrast', 'TrackerBar')
     # brightness = cv2.getTrackbarPos('Brightness', 'TrackerBar')
     img = img * (contrast/127 + 1) - contrast + brightness
     img = np.clip(img, 0, 255)
     img = np.uint8(img)
     return img

def thresholding(img, initialHSV):
     # picture = img
     contrast = initialHSV[6]
     brightness = initialHSV[7]
     # contrast = cv2.getTrackbarPos('Contrast', 'TrackerBar')
     # brightness = cv2.getTrackbarPos('Brightness', 'TrackerBar')
     img = img * (contrast/127 + 1) - contrast + brightness
     img = np.clip(img, 0, 255)
     img = np.uint8(img)
     # cv2.imshow('con', img)
     imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
     h_min = cv2.getTrackbarPos('HUE Min', 'TrackerBar')
     h_max = cv2.getTrackbarPos('HUE Max', 'TrackerBar')
     s_min = cv2.getTrackbarPos('SAT Min', 'TrackerBar')
     s_max = cv2.getTrackbarPos('SAT Max', 'TrackerBar')
     v_min = cv2.getTrackbarPos('VALUE Min', 'TrackerBar')
     v_max = cv2.getTrackbarPos('VALUE Max', 'TrackerBar')
     lowerWhite = np.array([initialHSV[0], initialHSV[2], initialHSV[4]]) # [h_min, s_min, v_min][initialHSV[0], initialHSV[1], initialHSV[2]]
     upperWhite = np.array([initialHSV[1], initialHSV[3], initialHSV[5]]) # [h_max, s_max, v_max][initialHSV[3], initialHSV[4], initialHSV[5]]
     maskWhite = cv2.inRange(imgHSV, lowerWhite, upperWhite)
     # print(initialHSV[0], initialHSV[1], initialHSV[2])

     return maskWhite

def warpImg(img, points, w, h, inv = False):
     # picture = img
     pts1 = np.float32(points)
     pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
     # if inv:
     #      matrix = cv2.getPerspectiveTransform(pts2, pts1)
     # else:
     matrix = cv2.getPerspectiveTransform(pts1, pts2)
     imgwarp = cv2.warpPerspective(img, matrix, (w, h))
     return imgwarp

def passFunction(x):
     pass

def initializeTrackerBar(initializeTrackerBarValues, wT = 480, hT = 240):
     windowName = 'TrackerBar'
     cv2.namedWindow(windowName)
     cv2.resizeWindow(windowName, 640, 480)
     cv2.createTrackbar('Width Top', windowName, initializeTrackerBarValues[0], wT//2, passFunction)
     cv2.createTrackbar('Height Top', windowName, initializeTrackerBarValues[1], hT, passFunction)
     cv2.createTrackbar('Width Bottom', windowName, initializeTrackerBarValues[2], wT//2, passFunction)
     cv2.createTrackbar('Height Bottom', windowName, initializeTrackerBarValues[3], hT, passFunction)
     # windowName1 = 'ColorPicker'
     # cv2.namedWindow(windowName)
     cv2.createTrackbar('HUE Min', windowName, 0, 255, passFunction)
     cv2.createTrackbar('HUE Max', windowName, 255, 255, passFunction)
     cv2.createTrackbar('SAT Min', windowName, 0, 255, passFunction)
     cv2.createTrackbar('SAT Max', windowName, 255, 255, passFunction)
     cv2.createTrackbar('VALUE Min', windowName, 0, 255, passFunction)
     cv2.createTrackbar('VALUE Max', windowName, 255, 255, passFunction)
     cv2.createTrackbar('Contrast', windowName, 0, 100, passFunction)
     cv2.createTrackbar('Brightness', windowName, 0, 100, passFunction)
     cv2.createTrackbar('rv', windowName, 0, 100, passFunction)
     cv2.createTrackbar('lv', windowName, 0, 100, passFunction)

def ValTrackers(wT = 480, hT = 240):
     windowName = 'TrackerBar'
     widthTop = cv2.getTrackbarPos('Width Top', windowName)
     heightTop = cv2.getTrackbarPos('Height Top', windowName)
     widthBottom = cv2.getTrackbarPos('Width Bottom', windowName)
     heightBottom = cv2.getTrackbarPos('Height Bottom', windowName)
     # widthTop = initialTrackerBarVals[0]
     # heightTop = initialTrackerBarVals[1]
     # widthBottom = initialTrackerBarVals[2]
     # heightBottom = initialTrackerBarVals[3]
     points = np.float32([(widthTop, heightTop), (wT - widthTop, heightTop), (widthBottom, heightBottom), (wT - widthBottom, heightBottom)])

     return points

def drawpoint(img, points):
     picture = img
     for i in range(4):
          cv2.circle(picture, (int(points[i][0]), int(points[i][1])), 15, (0, 0, 255), cv2.FILLED)
     
     return picture

def getHistogram(img, minPer=0.1, display = False, region = 1):
     if region == 1:
          histValues = np.sum(img, axis=0)
     else:
          histValues = np.sum(img[img.shape[0]//region: :], axis=0)
     # print(histValues)
     maxValue = np.max(histValues)
     minValue = minPer*maxValue

     indexArray = np.where(histValues >= minValue)
     basePoint = np.average(indexArray)
     # print(basePoint)

     if display:
          imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
          for x, intensity in enumerate(histValues):
               # print(img.shape[0] - intensity//255)x, img.shape[0] - intensity//255
               # cv2.line(imgHist, (x, img.shape[0]), (x, round(img.shape[0] - intensity//255//region)), (255, 0, 255), 1)
               cv2.circle(imgHist, (int(round(basePoint)), img.shape[0]), 20, (0, 255, 255), cv2.FILLED)
               # print(round(basePoint))
               return basePoint, imgHist
     return basePoint

def getV():
     rv = cv2.getTrackbarPos('rv', 'TrackerBar') / 100
     lv = cv2.getTrackbarPos('lv', 'TrackerBar') / 100
     return  -rv,lv

'''def getArea(img):

     area = np.sum(img)
     
     return area'''