# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Bool
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 as cv # OpenCV library
from adafruit_servokit import ServoKit
import busio

from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor
import numpy as np
import time
 
class Lane_Detection(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('lane_detection')
    detect_green = Detect_Green_L()
    while detect_green.is_green != True:
      rclpy.spin_once(detect_green)
    detect_green.destroy_node()
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages. 
    self.subscription = self.create_subscription(
      Image, 
      'video_frames', 
      self.control_degree, 
      1)
    self.subscription_LR = self.create_subscription(
      Bool, 
      'is_detect_left_or_right', 
      self.listener_callback, 
      1)
    self.subscriptionL = self.create_subscription(
      Bool, 
      'is_detect_left', 
      self.callbackL, 
      1)
    self.subscriptionR = self.create_subscription(
      Bool, 
      'is_detect_right', 
      self.callbackR, 
      1)
    self.subscription_stop = self.create_subscription(
      Bool, 
      'is_intersection_finished', 
      self.stop_callback, 
      1)
    #self.subscription_LR = self.create_subscription(
     # Bool, 
     # 'is_intersection_finished', 
     # self.dectect_callback, 
     # 1)
    self.subscription # prevent unused variable warning
    self.is_intersection = False
    self.is_LR = False
    self.Llist = []
    self.Rlist = []
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    self.i2c = busio.I2C(SCL, SDA)

    self.kit = ServoKit(channels = 16)

    self.pca = PCA9685(self.i2c, address=0x40)
    self.pca.frequency = 100

    self.pca.channels[15].duty_cycle = 0xFFFF
    self.pca.channels[9].duty_cycle = 0xFFFF

    self.motorR = motor.DCMotor(self.pca.channels[14], self.pca.channels[13])
    self.motorL = motor.DCMotor(self.pca.channels[10], self.pca.channels[11])
    self.motorR.throttle = 0.2
    self.motorL.throttle = -0.2
  
    self.angle = 80
  def stop_callback(self, msg):
    if msg.data == True:
      self.is_intersection = True
      print('True')  
  # def dectect_callback(self, msg):
  #   if msg.data == True:
  #     self.is_intersection = True
  def listener_callback(self, msg):
    """
    Callback function.
    """
    print(msg.data)
    # Display the message on the console
    if msg.data == True:
      self.is_LR = True
  def slice_frame (self, frame, edges, h):

    a1=np.zeros([640])
    for i in range (640):
        if edges[h,i]>0:
            a1[i]=1
    
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
              last=l
              pre=i
          l=i
                
    return pre,int((pre+last)/2),last

  def control_degree(self, msg):
    frame = self.br.imgmsg_to_cv2(msg)
    if self.is_LR is False and self.is_intersection is False:
      scale = 1
      delta = 0
      ddepth = cv.CV_16S
      frame = cv.resize(frame, (640, 480))

      try:
          #if self.is_intersection == True:
           # self.motorR.throttle = 0.2
           # self.motorL.throttle = -0.2
          self.angle = 80
          self.motorR.throttle = 0.3
          self.motorL.throttle = -0.3
          frame1=frame.copy()
          
          src = cv.GaussianBlur(frame, (3, 3), 0)
                
          gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
          grad_x = cv.Sobel(gray, ddepth, 1, 0, ksize=3, scale=scale, delta=delta, borderType=cv.BORDER_DEFAULT)
          grad_y = cv.Sobel(gray, ddepth, 0, 1, ksize=3, scale=scale, delta=delta, borderType=cv.BORDER_DEFAULT)
                
                
          abs_grad_x = cv.convertScaleAbs(grad_x)
          abs_grad_y = cv.convertScaleAbs(grad_y)
                
                
          grad = cv.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
                
          edges = cv.Canny(grad,100,200)
                
          arr0=[0,0,0,0,0]
          arr1=[0,0,0,0,0]
          arr2=[0,0,0,0,0]
          arr0[0],arr1[0],arr2[0] = self.slice_frame(frame1,edges,400)
          arr0[1],arr1[1],arr2[1] = self.slice_frame(frame1,edges,405)
          arr0[2],arr1[2],arr2[2] = self.slice_frame(frame1,edges,410)
          arr0[3],arr1[3],arr2[3] = self.slice_frame(frame1,edges,415)
          arr0[4],arr1[4],arr2[4] = self.slice_frame(frame1,edges,420)
                
          M=np.mean(arr1)
          M_i=0
          for i in range(len(arr1)):
              if arr1[i]==M:
                  M_i=i

          cv.line(frame, (0, 410), (640, 410), (0, 0, 255), 5)
          cv.circle(frame,(int(arr1[M_i]), 410), 10, (255, 0, 255), 3)
          cv.circle(frame,(int(640/2), 410), 5, (255, 0, 255), 3)
                
          cv.circle(frame,(int(arr0[M_i]), 410), 10, (255, 255, 0), 3)
          cv.circle(frame,(int(arr2[M_i]), 410), 10, (255, 255, 0), 3)
                
                
          self.angle +=  (((int(arr1[M_i]) - 320) / 5) * 2)
          print(self.angle)
        #   if self.angle < 70 and self.angle > 40:
        #       self.angle = 80
          if self.angle < 40:
              self.angle = 20
              self.motorR.throttle = 1
              self.motorL.throttle = 0.2
          elif self.angle > 135:
              self.angle = 165
              self.motorR.throttle = -0.2
              self.motorL.throttle = -1
                
          self.kit.servo[7].angle = self.angle         
            # cv.imshow('grad', edges)
                
            # 顯示圖片
          #cv.imshow('frame', frame) 
          #cv.waitKey(1)
      except KeyboardInterrupt:
          self.motorR.throttle = 0
          self.motorL.throttle = 0

          self.pca.deinit()
    elif self.is_LR:
      # cv.imshow('frame', frame) 
      # cv.waitKey(1)
      self.motorR.throttle = 0
      self.motorL.throttle = 0
      self.angle = 80
      
      if len(self.Llist) + len(self.Rlist) >= 5:
        print(len(self.Llist) + len(self.Rlist))
        if len(self.Llist) > len(self.Rlist):
          self.motorR.throttle = 0
          self.motorL.throttle = 0
          self.angle = 80
          self.kit.servo[7].angle = self.angle
          
        else:
          self.motorR.throttle = 0
          self.motorL.throttle = 0
          self.angle = 80
          self.kit.servo[7].angle = self.angle
          self.turnright()
          self.is_LR = False
    else:
      # if len(self.Llist) + len(self.Rlist) >= 5:
      #   print(len(self.Llist) + len(self.Rlist))
      if len(self.Llist) > len(self.Rlist):
        time.sleep(0.4)
        self.motorR.throttle = 0
        self.motorL.throttle = 0
        self.kit.servo[7].angle = 165
        time.sleep(1)
        self.motorR.throttle = 0.25
        self.motorL.throttle = -1
        time.sleep(1)
        self.is_intersection = False
      print(self.angle)

  def callbackL(self, msg):
    if msg.data == True:
      self.Llist.append(msg.data)
  def callbackR(self, msg):
    if msg.data == True:
      self.Rlist.append(msg.data)
  def turnright(self):
    print("step 0")
    time.sleep(1)
    self.motorR.throttle = -0.6
    self.motorL.throttle = 0.6
    self.kit.servo[7].angle = self.angle
    time.sleep(0.37)
    print("step 1")
    self.kit.servo[7].angle = 80
    self.motorR.throttle = 0
    self.motorL.throttle = -0
    time.sleep(0.5)
    print("step 2")
    self.kit.servo[7].angle = 165
    self.motorR.throttle = 0.25
    self.motorL.throttle = -1
    time.sleep(0.87)
    print("step 3")
    self.kit.servo[7].angle = 80
    self.motorR.throttle = 0
    self.motorL.throttle = -0
    print("step 4")
class Detect_Green_L(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('detect_greenL')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.is_green = False
    self.subscription = self.create_subscription(
      Bool, 
      'is_traffic_light_finished', 
      self.listener_callback, 
      1)
    
    self.subscription # prevent unused variable warning
    # print(self.is_green)
    # Used to convert between ROS and OpenCV images
   
  def listener_callback(self, msg):
    """
    Callback function.
    """
    # print(self.is_green)
    # Display the message on the console
    if msg.data == True:
      self.is_green = True
      self.get_logger().info('Receiving True')
    # self.get_logger().info('Receiving False')
 
    # self.get_logger().info('Receiving Boolean')

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  lane_detection = Lane_Detection()
  rclpy.spin(lane_detection)
  # Spin the node so the callback function is called.
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  lane_detection.destroy_node()
  # print("killed")
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
