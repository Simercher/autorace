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
import cv2 # OpenCV library
import os
import numpy as np

MIN_MATCH_COUNT = 8
 
class Detect_left_or_right(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('detect_left_or_right')

    self.imgL = cv2.imread('/home/ai-club/autorace_ws/src/cv_basics/cv_basics/picture/left.png')
    self.imgL = cv2.resize(self.imgL, (50, 50))
    self.imgR = cv2.imread('/home/ai-club/autorace_ws/src/cv_basics/cv_basics/picture/right.png')
    self.imgR = cv2.resize(self.imgR, (50, 50))
    self.siftL = cv2.SIFT_create()
    self.siftR = cv2.SIFT_create()
    kp1, self.des1 = self.siftL.detectAndCompute(self.imgL,None)
    kp1, self.des = self.siftR.detectAndCompute(self.imgR,None)
    self.msg = Bool()
    self.msg.data = False
    self.i = 0
    self.is_green = False
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    # detect_intersection_finished = Detect_intersection_finished()
    print("step 1")
    # while detect_intersection_finished.is_intersection_finished_ != True:
    #   rclpy.spin_once(detect_intersection_finished)

    # detect_intersection_finished.destroy_node()
    self.subscription = self.create_subscription(
      Bool, 
      'is_traffic_light_finished', 
      self.listener_callback, 
      1)
    self.subscription = self.create_subscription(
      Image, 
      'video_frames', 
      self.feature_matching, 
      1)
    self.subscription # prevent unused variable warning
    self.publisher_ = self.create_publisher(Bool, 'is_detect_left_or_right', 1)
    self.publisherL = self.create_publisher(Bool, 'is_detect_left', 1)
    self.publisherR = self.create_publisher(Bool, 'is_detect_right', 1)
    # Used to convert between ROS and OpenCV images
    print("step 2")
    self.br = CvBridge()

  def listener_callback(self, msg):
    """
    Callback function.
    """
    # print(self.is_green)
    # Display the message on the console
    # print("test")
    # print("step 1-2")
    if msg.data == True:
      self.is_green = True
      self.get_logger().info('Receiving True')
    else:
      self.get_logger().info('Receiving False')

  def findLeft(self, frame):
    try:
      kp2, des2 = self.siftL.detectAndCompute(frame,None)
      FLANN_INDEX_KDTREE = 1
      index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
      search_params = dict(checks = 50)
      flann = cv2.FlannBasedMatcher(index_params, search_params)
      matches = flann.knnMatch(self.des1,des2,k=2)
      # store all the good matches as per Lowe's ratio test.
      good = []
      for m,n in matches:
          if m.distance < 0.75 * n.distance:
            good.append(m)
    except :
      good = []
    if len(good)>MIN_MATCH_COUNT:
        self.msg.data = True
        if self.i < 5:
          self.publisher_.publish(self.msg)
          self.publisherL.publish(self.msg)
          print('Left : Yes')
          self.i += 1
    else:
        # print( "Left Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
        matchesMask = None
  def findRight(self, frame):
    try :  
      kp2, des2 = self.siftR.detectAndCompute(frame,None)
      FLANN_INDEX_KDTREE = 1
      index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
      search_params = dict(checks = 50)
      flann = cv2.FlannBasedMatcher(index_params, search_params)
      matches = flann.knnMatch(self.des,des2,k=2)
      # store all the good matches as per Lowe's ratio test.
      good = []
      for m,n in matches:
          if m.distance < 0.75 * n.distance:
            good.append(m)
    except:
      good = []

    if len(good)>MIN_MATCH_COUNT:
        self.msg.data = True
        if self.i < 5:
          self.publisher_.publish(self.msg)
          self.publisherR.publish(self.msg)
          print('Right : Yes')
          self.i += 1
    else:
        # print( "Right Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
        matchesMask = None
  def feature_matching(self, data):
    if self.is_green == True:
      frame = self.br.imgmsg_to_cv2(data)
      frame = cv2.resize(frame, (200, 200))
      frameOrignal = frame.copy()
      # print("is work")
      self.findLeft(frame)
      self.findRight(frame)
      # print("step 3")

class Detect_intersection_finished(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('detect_intersection_finished')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    # print("step 1-1")
    self.is_intersection_finished_ = False
    self.subscription = self.create_subscription(
      Bool, 
      'is_traffic_light__finished', 
      self.listener_callback, 
      1)
    
    self.subscription # prevent unused variable warning
    # print(self.is_green)
    # Used to convert between ROS and OpenCV images
   

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  detect_left_or_right = Detect_left_or_right()
  
  # Spin the node so the callback function is called.
  rclpy.spin(detect_left_or_right)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  detect_left_or_right.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
