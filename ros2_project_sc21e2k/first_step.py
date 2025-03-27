
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
    def __init__(self, stop):
        super().__init__('cI')
        self.get_logger().info("ColourIdentifier node initialized.")
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sensitivity = 20
        self.foundTarget = stop
        self.lastCheck = time.time()
        self.CHECK_PERIOD = 1.0

        
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
        red_mask = cv2.bitwise_or(red_mask1, red_mask2 )
        rb_mask  = cv2.bitwise_or(red_mask , blue_mask )
        rg_mask  = cv2.bitwise_or(red_mask , green_mask)
        bg_mask  = cv2.bitwise_or(blue_mask, green_mask)
        rgb_mask = cv2.bitwise_or(rb_mask  , green_mask)

        # filtered_image = cv2.bitwise_and(image, image, mask=cv2.bitwise_or(blue_mask, blue_mask))
        filtered_image = image
        cv2.namedWindow("camera feed", cv2.WINDOW_NORMAL)
        cv2.imshow("camera feed", filtered_image)
        cv2.resizeWindow("camera feed", 320, 240)
        cv2.waitKey(3)

        if (time.time() - self.lastCheck > self.CHECK_PERIOD):
            self.lastCheck = time.time()
            self.isTarget(cv2.bitwise_and(image, image, mask=cv2.bitwise_or(blue_mask, blue_mask)), cv2.bitwise_or(blue_mask, blue_mask))


    def isTarget(self, image, mask):
        contours, _ = cv2.findContours(mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        if (contours is not None and len(contours) > 0):
            self.get_logger().info("FOUND TARGET")
            largest = max(contours, key=cv2.contourArea)
            if (cv2.contourArea(largest) > 500):
                moment = cv2.moments(largest)
                if (moment["m00"] != 0):
                    centerx = moment["m10"] / moment["m00"]
                    centery = moment["m01"] / moment["m00"]
                    goalMessage = Twist()
                    goalMessage.linear.x = 0.2
                    goalMessage.angular.z = 0.0
                    self.get_logger().info("Publishing target")
                    self.publisher.publish(goalMessage)
                
            self.foundTarget[0] = True
            return 1
        return 0



class GoToPose(Node):
    def __init__(self, foundTarget):
        super().__init__('navigation_goal_action_client')
        self.get_logger().info("GoToPose node initialized.")
        self.action_client      = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.subscriber         = self.create_subscription(Twist, '/cmd_vel', self.moveToTarget, 10)
        self.foundTarget        = foundTarget
        self.currentPosition    = None
        self.currentOrientation = None
        self.goalHandler        = None
        self.lastGoalUpdate     = time.time()

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

    def cancel_goal(self):
        if (self.goalHandler is not None):
            cancelFuture = self.goalHandler.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancelFuture)
            self.goalHandler = None

    def search(self):
        # Start by making random moves within bounds of the room - (7.28, 5.42), (8.78, -13.1), (-9.54, -14.7), (-11.1, 3.89)
        SEARCH_TIME = 30
        TARGET_DETECTION_RATE = 0.05
        while not self.foundTarget[0]:
            randx = round(random.uniform(-9.54, 7.28), 2)
            randy = round(random.uniform(-13.1, 3.89), 2)
            self.send_goal(randx, randy, 0.0)
            time.sleep(TARGET_DETECTION_RATE * 2)
            for i in range(0, int(SEARCH_TIME/TARGET_DETECTION_RATE)-2):
                if (self.currentPosition is not None):
                    self.get_logger().info(f"FOUND TARGET: {self.foundTarget[0]}   CURRENT x: {self.currentPosition.x}   CURRENT y: {self.currentPosition.y}")
                if (self.foundTarget[0] and self.currentPosition is not None):
                    self.get_logger().info("CALLING CANCEL GOAL")
                    # self.cancel_goal()
                    break
                else: time.sleep(TARGET_DETECTION_RATE)
        self.get_logger().info("Stopped due to finding target")


    def moveToTarget(self, data):
        if (self.currentPosition is not None and data.linear is not None and self.foundTarget[0]):
            self.get_logger().info("Moving Towards Target")
            dt = time.time() - self.lastGoalUpdate
            if (dt > 2):
                self.lastGoalUpdate = time.time()

                linearVelocity = data.linear.x
                angularVelocity = data.angular.z
                dx = linearVelocity * cos(self.currentPosition.z) * dt
                dy = linearVelocity * sin(self.currentPosition.z) * dt
                dTheta = angularVelocity * dt
                self.send_goal(self.currentPosition.x + dx, self.currentPosition.y + dy, self.currentPosition.z + dTheta)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        self.goalHandler = goal_handle

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.goalHandler = None

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # ## Access the current pose
        current_pose = feedback_msg.feedback.current_pose
        position = current_pose.pose.position
        orientation = current_pose.pose.orientation
        self.currentPosition = position
        self.currentOrientation = orientation

        # ## Access other feedback fields
        # navigation_time = feedback_msg.feedback.navigation_time
        # distance_remaining = feedback_msg.feedback.distance_remaining

        # ## Print or process the feedback data
        # self.get_logger().info(f'Current Pose: [x: {position.x}, y: {position.y}, z: {position.z}]')
        # self.get_logger().info(f'Distance Remaining: {distance_remaining}')

        
def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()

    print("In Main")
    rclpy.init(args=None)

    # In a list so that python treats it as a reference between classes (so one can change the value in the other)
    foundTarget = [False]   
    cI = colourIdentifier(foundTarget)
    mover = GoToPose(foundTarget)

    signal.signal(signal.SIGINT, signal_handler)
    executor = MultiThreadedExecutor()
    executor.add_node(cI)
    executor.add_node(mover)
    executorThread = threading.Thread(target=executor.spin, daemon=True)
    executorThread.start()

    time.sleep(1)
    mover.search()

    try:
        while rclpy.ok():
            continue
    except ROSInterruptException:
        pass

    cv2.destroyAllWindows()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
