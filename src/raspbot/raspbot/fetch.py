# Modified from code taken from guyuehome.com

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge
import cv2
import imutils

greenLower = (60, 60, 60)
greenUpper = (90, 255, 255)

"""
创建一个订阅者节点
"""
class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub = self.create_subscription(
            Image, 'rare', self.listener_callback, 10)
        self.cv_bridge = CvBridge()
        self.publisher = self.create_publisher(Int16MultiArray, 'ball_tracking', 10)
        self.chase = False

    def object_detect(self, image):
        blurred = cv2.GaussianBlur(image, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)

        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        msg = Int16MultiArray()
        msg.data = [-1, -1, -1, 0]

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            self.get_logger().info(f"x: {int(x)}, y: {int(y)}, r: {int(radius)}")

            if radius > 10:
                cv2.circle(image, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image, center, 5, (0, 0, 255), -1)
                msg.data = [int(x), int(y), int(radius), int(self.chase)]
        key = cv2.waitKey(1)
        # if space is pushed:
        if key == 32:
            self.chase = not self.chase
        cv2.putText(image, f"Following: {self.chase}", (50, 50), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0))
        cv2.imshow("Image", image)
        cv2.imshow("Mask", mask)
        self.publisher.publish(msg)


    def listener_callback(self, data):
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        self.object_detect(image)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ImageSubscriber("topic_webcam_sub")
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("Exiting")
