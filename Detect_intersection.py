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

MIN_MATCH_COUNT = 10
 
class Detect_intersection(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('detect_intersection')

    self.img = cv2.imread('/home/ai-club/autorace_ws/src/cv_basics/cv_basics/picture/stop.png')
    self.img = cv2.resize(self.img, (50, 50))
    self.sift = cv2.SIFT_create()
    self.kp1, self.des1 = self.sift.detectAndCompute(self.img,None)
    detect_green = Detect_Green()
    while detect_green.is_green != True:
      rclpy.spin_once(detect_green)
    detect_green.destroy_node()
    self.msg = Bool()
    self.msg.data = False
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      'video_frames', 
      self.feature_matching, 
      1)
    self.subscription # prevent unused variable warning
    self.publisher_ = self.create_publisher(Bool, 'is_intersection_finished', 1)
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    
  def findID(self, frame):
    try:
      kp2, des2 = self.sift.detectAndCompute(frame,None)
      FLANN_INDEX_KDTREE = 1
      index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
      search_params = dict(checks = 50)
      flann = cv2.FlannBasedMatcher(index_params, search_params)
      matches = flann.knnMatch(self.des1,des2,k=2)
      # store all the good matches as per Lowe's ratio test.
      good = []
      for m,n in matches:
          if m.distance < 0.845 * n.distance:
            good.append(m)
    except :
      good = []
    if len(good)>MIN_MATCH_COUNT:
      print("is detet intersection")
      self.msg.data = True
      self.publisher_.publish(self.msg)
    else:
      print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
        # matchesMask = None
      # self.publisher_.publish(self.msg)
  def feature_matching(self, data):
    frame = self.br.imgmsg_to_cv2(data)
    frame = cv2.resize(frame, (200, 200))
    frameOrignal = frame.copy()
    self.findID(frame)
    
    # cv2.imshow("camera", frameOrignal)
    # cv2.waitKey(1)

class Detect_Green(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('detect_green')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.is_green = False
    self.subscription = self.create_subscription(
      Bool, 
      'is_detect_left_or_right', 
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
    self.get_logger().info('Receiving False')

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  detect_intersection = Detect_intersection()
  
  while True:
    rclpy.spin_once(detect_intersection)
    if detect_intersection.msg.data == True:
      break
  # Spin the node so the callback function is called.
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  detect_intersection.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
