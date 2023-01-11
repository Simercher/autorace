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
import numpy as np
 
class Traffic_light(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('traffic_light')
    self.HSVVals = [33, 92, 103, 255, 63, 255, 0, 0]
    self.is_detect_green = False
    self.is_finished = False
    self.area = 0
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    print("step 1")
    self.subscription = self.create_subscription(
      Image, 
      'video_frames', 
      self.detect_green, 
      1)
    # self.subscription # prevent unused variable warning

    self.publisher_ = self.create_publisher(Bool, 'is_traffic_light_finished', 1)
    print("step 2")
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def detect_green(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    # self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    self.get_logger().info('Receiving Frame')
    frame = self.br.imgmsg_to_cv2(data)
    #cv2.imshow("camera", frame)
    #cv2.waitKey(1)
    mask = self.thresholding(frame, self.HSVVals)
    self.area = self.Area_sum(mask)
    msg = Bool()
    msg.data = False

    print(self.area)
    if self.area > 20000:
      msg.data = True
      self.publisher_.publish(msg)
    self.publisher_.publish(msg)
    # print(current_frame)
    # print(data)
    # Display image
    # cv2.imshow("camera", frame)
    
    # cv2.waitKey(1)
  def thresholding(self, img, initialHSV):
     # picture = img
     contrast = initialHSV[6]
     brightness = initialHSV[7]
     img = img * (contrast/127 + 1) - contrast + brightness
     img = np.clip(img, 0, 255)
     img = np.uint8(img)
     # cv2.imshow('con', img)
     imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
     lowerWhite = np.array([initialHSV[0], initialHSV[2], initialHSV[4]]) # [h_min, s_min, v_min][initialHSV[0], initialHSV[1], initialHSV[2]]
     upperWhite = np.array([initialHSV[1], initialHSV[3], initialHSV[5]]) # [h_max, s_max, v_max][initialHSV[3], initialHSV[4], initialHSV[5]]
     maskWhite = cv2.inRange(imgHSV, lowerWhite, upperWhite)
     # print(initialHSV[0], initialHSV[1], initialHSV[2])

     return maskWhite
  def Area_sum(self, mask):
    area = np.sum(mask)
    return area
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  traffic_light = Traffic_light()
  
  # Spin the node so the callback function is called.
  while True:
    rclpy.spin_once(traffic_light)
    if traffic_light.area > 100000:
      print("True")
      break
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  traffic_light.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
