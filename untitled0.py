#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 28 16:22:36 2022

@author: york
"""

import cv2  as cv
import numpy as np
from adafruit_servokit import ServoKit
import busio

from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor


def slice(frame,edges,h):
    #cv.line(frame, (0, h), (640, h), (0, 0, 255), 5)
    #cv.line(frame, (0, 420), (640, 420), (0, 0, 255), 5)
    #cv.line(frame, (0, 440), (640, 440), (0, 0, 255), 5)
    
    
    a1=np.zeros([640])
    for i in range (640):
        if edges[h,i]>0:
            a1[i]=1
            
            #cv.circle(frame,(i, h), 10, (0, 255, 255), 3)
    # print(a1)
    
    a1[0]=1
    a1[639]=1
    
    l=0
    pre=0
    last=0
    max=0
    for i in range(640):
       if a1[i]==1:
           
           if i-l>max:
               max=i-l
            #    print('!')
               last=l
               pre=i
           l=i
           
            
    
    #cv.circle(frame,(int(pre), h), 10, (255, 255, 0), 3)
    #cv.circle(frame,(int(last), h), 10, (255, 255, 0), 3)
    
    
    #cv.circle(frame,(int((pre+last)/2), h), 10, (255, 0, 255), 3)
    #cv.circle(frame,(int((640)/2), h), 5, (255, 0, 255), 3)
    
    return pre,int((pre+last)/2),last
    

def main():
    # 選擇第二隻攝影機
    cap = cv.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)
    scale = 1
    delta = 0
    ddepth = cv.CV_16S
    angle = 80

    i2c = busio.I2C(SCL, SDA)

    kit = ServoKit(channels = 16)

    pca = PCA9685(i2c, address=0x40)
    pca.frequency = 100

    pca.channels[15].duty_cycle = 0xFFFF
    pca.channels[9].duty_cycle = 0xFFFF

    motorR = motor.DCMotor(pca.channels[14], pca.channels[13])
    motorL = motor.DCMotor(pca.channels[10], pca.channels[11])
    motorR.throttle = 0.6
    motorL.throttle = -0.6

    try:
        while(True):
            angle = 80
            # 從攝影機擷取一張影像
            ret, frame = cap.read()
        
            frame1=frame
        
            src = cv.GaussianBlur(frame, (3, 3), 0)
            
            gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
            
            grad_x = cv.Sobel(gray, ddepth, 1, 0, ksize=3, scale=scale, delta=delta, borderType=cv.BORDER_DEFAULT)
            # Gradient-Y
            # grad_y = cv.Scharr(gray,ddepth,0,1)
            grad_y = cv.Sobel(gray, ddepth, 0, 1, ksize=3, scale=scale, delta=delta, borderType=cv.BORDER_DEFAULT)
            
            
            abs_grad_x = cv.convertScaleAbs(grad_x)
            abs_grad_y = cv.convertScaleAbs(grad_y)
            
            
            grad = cv.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
            
            edges = cv.Canny(grad,100,200)
            
            arr0=[0,0,0,0,0]
            arr1=[0,0,0,0,0]
            arr2=[0,0,0,0,0]
            arr0[0],arr1[0],arr2[0]=slice(frame1,edges,400)
            arr0[1],arr1[1],arr2[1]=slice(frame1,edges,405)
            arr0[2],arr1[2],arr2[2]=slice(frame1,edges,410)
            arr0[3],arr1[3],arr2[3]=slice(frame1,edges,415)
            arr0[4],arr1[4],arr2[4]=slice(frame1,edges,420)
            
            M=np.mean(arr1)
            M_i=0
            for i in range(len(arr1)):
                if arr1[i]==M:
                    M_i=i
                    
            
            # print(int(arr1[M_i]))
            cv.line(frame, (0, 410), (640, 410), (0, 0, 255), 5)
            cv.circle(frame,(int(arr1[M_i]), 410), 10, (255, 0, 255), 3)
            cv.circle(frame,(int(640/2), 410), 5, (255, 0, 255), 3)
            
            cv.circle(frame,(int(arr0[M_i]), 410), 10, (255, 255, 0), 3)
            cv.circle(frame,(int(arr2[M_i]), 410), 10, (255, 255, 0), 3)
            
            
            angle +=  (((int(arr1[M_i]) - 320) / 5) * 2)
            print(angle)

            if angle < 70 and angle > 40:
              angle = 80
            elif angle < 40:
                angle = 20
                motorR.throttle = 0.65
                motorL.throttle = -0.3
            elif angle > 135:
                angle = 165
                motorR.throttle = 0.25
                motorL.throttle = -1
            
            kit.servo[7].angle = angle
            #cv.line(frame, (0, 0), (640, 480), (0, 0, 255), 5)
            #cv.line(frame, (0, 480), (640, 480), (0, 0, 255), 5)
            
            
            cv.imshow('grad', edges)
            
            # 顯示圖片
            cv.imshow('frame', frame)
            
            # 若按下 q 鍵則離開迴圈
            if cv.waitKey(1) & 0xFF == ord('q'):
                motorR.throttle = 0
                motorL.throttle = 0

                pca.deinit()
                break
    except KeyboardInterrupt:
        motorR.throttle = 0
        motorL.throttle = 0

        pca.deinit()

    # 釋放攝影機
    cap.release()

    # 關閉所有 OpenCV 視窗
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()