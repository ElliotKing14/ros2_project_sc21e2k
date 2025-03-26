
import threading
import sys, time
import cv2
import numpy as np
import signal
import random
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import MultiThreadedExecutor
from rclpy.exceptions import ROSInterruptException
from math import sin, cos


class colourIdentifier(Node):
    def __init__(self):
        super().__init__('cI')
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.callback, 10)
        self.subscription
        self.sensitivity = 20

        
    def callback(self, data):
        cvBridge = CvBridge()
        try: 
                image = cvBridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError(e):
                print(e)

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

        # filtered_image = cv2.bitwise_and(image, image, mask=rgb_mask)
        filtered_image = hsv_image
        cv2.namedWindow("camera feed", cv2.WINDOW_NORMAL)
        cv2.imshow("camera feed", filtered_image)
        cv2.resizeWindow("camera feed", 320, 240)
        cv2.waitKey(3)


class GoToPose(Node):
    def __init__(self):
        super().__init__('navigation_goal_action_client')
        self.get_logger().info("GoToPose node initialized.")
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw):
        self.get_logger().info(f"Sending goal: x = {x}, y = {y}, yaw = {yaw}")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        ## Access the current pose
        current_pose = feedback_msg.feedback.current_pose
        position = current_pose.pose.position
        orientation = current_pose.pose.orientation

        ## Access other feedback fields
        navigation_time = feedback_msg.feedback.navigation_time
        distance_remaining = feedback_msg.feedback.distance_remaining

        ## Print or process the feedback data
        self.get_logger().info(f'Current Pose: [x: {position.x}, y: {position.y}, z: {position.z}]')
        self.get_logger().info(f'Distance Remaining: {distance_remaining}')

        
def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()

    print("In Main")
    rclpy.init(args=None)
    cI = colourIdentifier()
    mover = GoToPose()

    signal.signal(signal.SIGINT, signal_handler)
    executor = MultiThreadedExecutor()
    executor.add_node(cI)
    executor.add_node(mover)
    signal.signal(signal.SIGINT, signal_handler)
    executorThread = threading.Thread(target=executor.spin, daemon=True)
    executorThread.start()

    # randx = random.randint(-5.0, 5.0)
    # randy = random.randint(-5.0, 5.0)
    time.sleep(1)
    mover.send_goal(-5.0, -5.0, 0.0)

    try:
        while rclpy.ok():
            continue
    except ROSInterruptException:
        pass

    cv2.destroyAllWindows()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
