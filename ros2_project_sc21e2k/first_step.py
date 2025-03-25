# Exercise 1 - Display an image of the camera feed to the screen

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class colourIdentifier(Node):
    def __init__(self):
        super().__init__('cI')
        
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data

        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.callback, 10)
        self.subscription
        self.sensitivity = 20

        
    def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        # Show the resultant images you have created.
        cvBridge = CvBridge()
        try: 
                image = cvBridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError(e):
                print(e)

        # Create view with green mask
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv_green_lower = np.array([60 -self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 +self.sensitivity, 255, 255])
        hsv_red_lower1 =  np.array([0                   , 100, 100])
        hsv_red_lower2 =  np.array([180-self.sensitivity, 100, 100])
        hsv_red_upper1 =  np.array([    self.sensitivity, 255, 255])
        hsv_red_upper2 =  np.array([180                 , 255, 255])
        hsv_blue_lower =  np.array([120-self.sensitivity, 100, 100])
        hsv_blue_upper =  np.array([120+self.sensitivity, 255, 255])
        green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        red_mask1  = cv2.inRange(hsv_image, hsv_red_lower1,  hsv_red_upper1 )
        red_mask2  = cv2.inRange(hsv_image, hsv_red_lower2,  hsv_red_upper2 )
        blue_mask  = cv2.inRange(hsv_image, hsv_blue_lower,  hsv_blue_upper )
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        rb_mask = cv2.bitwise_or(red_mask,  blue_mask )
        rg_mask = cv2.bitwise_or(red_mask,  green_mask)
        bg_mask = cv2.bitwise_or(blue_mask, green_mask)
        rgb_mask= cv2.bitwise_or(rb_mask,   green_mask)

        filtered_image = cv2.bitwise_and(image, image, mask=rgb_mask)
        # contours, _ = cv2.findContours(filtered_image, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        # c = max(contours, key=cv2.contourArea)
        # if (c == None or c == 0): print("NOT FOUND")
        cv2.namedWindow("camera feed", cv2.WINDOW_NORMAL)
        cv2.imshow("camera feed", filtered_image)
        cv2.resizeWindow("camera feed", 320, 240)
        cv2.waitKey(3)
        

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():

    def signal_handler(sig, frame):
        rclpy.shutdown()
    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    cI = colourIdentifier()


    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(cI,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            continue
    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()
    

# Check if the node is executing in the main path
if __name__ == '__main__':
    main()
