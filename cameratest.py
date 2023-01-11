import cv2 # OpenCV library
import os
import numpy as np

MIN_MATCH_COUNT = 8
imgL = cv2.imread('/home/ai-club/autorace_ws/src/cv_basics/cv_basics/picture/left.png')
imgL = cv2.resize(imgL, (50, 50))
imgR = cv2.imread('/home/ai-club/autorace_ws/src/cv_basics/cv_basics/picture/right.png')
imgR = cv2.resize(imgR, (50, 50))
siftL = cv2.SIFT_create()
kp1, des1 = siftL.detectAndCompute(imgL,None)
siftR = cv2.SIFT_create()
kp, des = siftR.detectAndCompute(imgR,None)

def findLeft(imgL, frame):
     kp2, des2 = siftL.detectAndCompute(frame,None)
     FLANN_INDEX_KDTREE = 1
     index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
     search_params = dict(checks = 50)
     flann = cv2.FlannBasedMatcher(index_params, search_params)
     matches = flann.knnMatch(des1,des2,k=2)
     # store all the good matches as per Lowe's ratio test.
     good = []
     for m,n in matches:
         if m.distance < 0.75 * n.distance:
           good.append(m)

     if len(good)>MIN_MATCH_COUNT:
          print('Left : Yes')
     else:
     #    print( "Left Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
        matchesMask = None
def findRight(imgR, frame):
     
     kp2, des2 = siftR.detectAndCompute(frame,None)
     FLANN_INDEX_KDTREE = 1
     index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
     search_params = dict(checks = 50)
     flann = cv2.FlannBasedMatcher(index_params, search_params)
     matches = flann.knnMatch(des,des2,k=2)
     # store all the good matches as per Lowe's ratio test.
     good = []
     for m,n in matches:
         if m.distance < 0.8 * n.distance:
           good.append(m)

     if len(good)>MIN_MATCH_COUNT:
          print('Right : Yes')
          # self.i += 1
     else:
     #    print( "Right Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
        matchesMask = None
def feature_matching():
     cap = cv2.VideoCapture(0)
     cap.set(3, 640)
     cap.set(4, 480)
     while True:
          ret, frame = cap.read()
          frame = cv2.resize(frame, (200, 200))
          frameOrignal = frame.copy()
          findLeft(imgL, frame)
          findRight(imgR, frame)
          cv2.imshow('resizeframe', frame)
          cv2.imshow('L', imgL)
          cv2.imshow('R', imgR)    


def main(args=None):
     feature_matching()
  
  
if __name__ == '__main__':
  main()