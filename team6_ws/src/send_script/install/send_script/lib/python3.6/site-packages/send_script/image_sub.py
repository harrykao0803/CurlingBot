#!/usr/bin/env python
import rclpy

from rclpy.node import Node

import sys
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *

from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

from cv_bridge import CvBridge
import cv2

import numpy as np
import math


def process(src_img):

    # convert the input image into grayscale
    src_image = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)

    if src_image is None:
        sys.exit("Could not read the image.")

    # Preprocess the image to binary.
    blurred_image = cv2.GaussianBlur(src_image, (5, 5), 0)
    _, binary_image = cv2.threshold(blurred_image, 128, 255, cv2.THRESH_BINARY)

    # Perform morphological opening to reduce noise.
    binary_image = cv2.erode(binary_image, None, iterations=2)
    binary_image = cv2.dilate(binary_image, None, iterations=2)

    # cv2.imshow('grayscale image', binary_image)  # show the converted grayscale image
    # cv2.waitKey(0)                               # 暫停並等待任意按鍵命令
    # cv2.destroyAllWindows()                      # 關閉視窗

    # Find contours in the binary image.
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # print the number of detected objects
    print(len(contours))

    # Load the original image in color to display the results.
    color_image = cv2.cvtColor(src_image, cv2.COLOR_GRAY2BGR)

    result = []

    for contour in contours:

        xmin = (10000000, 0)
        xmax = (-10000000, 0)
        ymin = (0, 10000000)
        ymax = (0, -10000000)

        for pixel in contour:
            px, py = pixel[0][0], pixel[0][1]
            xmin = (px, py) if px < xmin[0] else xmin
            xmax = (px, py) if px > xmax[0] else xmax
            ymin = (px, py) if py < ymin[1] else ymin
            ymax = (px, py) if py > ymax[1] else ymax

        # print(f"xmax: {xmax}, ymax: {ymax}")

        # # cv2.circle(color_image, xmin, 5, (0, 0, 255), -1)
        # cv2.circle(color_image, xmax, 5, (0, 0, 255), -1)
        # # cv2.circle(color_image, ymin, 5, (0, 0, 255), -1)
        # cv2.circle(color_image, ymax, 5, (0, 0, 255), -1)

        # cv2.circle(color_image, (0, 0), 5, (255, 0, 255), -1)

        # Calculate moments to find the centroid and orientation.
        M = cv2.moments(contour)
        if M["m00"] < 100:  # Skip if the contour area is zero.
            continue

        # Calculate centroid (cx, cy).
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]

        # # Calculate orientation (principal angle).
        # angle = 0.5 * np.arctan2(2 * M["mu11"], (M["mu20"] - M["mu02"]))
        # angle_degrees = np.degrees(angle)

        angle = np.arctan2(xmax[1] - ymax[1], xmax[0] - ymax[0])
        angle_degrees = np.degrees(angle)

        # Draw the centroid on the image.
        cv2.circle(color_image, (int(cx), int(cy)), 5, (0, 255, 0), -1)

        # Draw the principal direction as a bidirectional line.
        length = 400  # Length of the line in one direction
        dx = int(length * np.cos(angle))
        dy = int(length * np.sin(angle))

        # Draw the line extending equally in both directions from the centroid
        cv2.line(color_image, (int(cx) - dx, int(cy) - dy), (int(cx) + dx, int(cy) + dy), (255, 0, 0), 2)

        # Display the centroid coordinates and principal angle.
        print(f"Centroid: ({cx}, {cy}), Principal Angle: {angle_degrees:.2f} degrees")

        result.append(cx)
        result.append(cy)
        result.append(angle_degrees)

    # Show the original image with marked centroids and angles.
    success = cv2.imwrite("ObjectDetection.jpg", color_image)
    if not success:
        print ("Error fuck that shit")
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    return result


class ImageSub(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(Image, 'techman_image', self.image_callback, 10)
        self.subscription
        self.publisher = self.create_publisher(Float32MultiArray, '/techman_block', 10)

    def image_callback(self, data):
        self.get_logger().info('Received image')

        # TODO (write your code here)
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(data)
        result = process(image)

        msg = Float32MultiArray()
        msg.data = result

        self.publisher.publish(msg)


def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()

def set_io(state):
    gripper_node = rclpy.create_node('gripper')
    gripper_cli = gripper_node.create_client(SetIO, 'set_io')

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not availabe, waiting again...')

    io_cmd = SetIO.Request()
    io_cmd.module = 1
    io_cmd.type = 1
    io_cmd.pin = 0
    io_cmd.state = state
    gripper_cli.call_async(io_cmd)
    gripper_node.destroy_node()


def main(args=None):

    # 初始化 ROS
    rclpy.init(args=args)

    # 創建一個叫做 image_sub 的 Node
    node = ImageSub('image_sub')

    # 讓 node (image_sub) 持續運行
    rclpy.spin(node)

    # 關閉 node (image_sub)
    node.destroy_node()

    # 關閉 ROS
    rclpy.shutdown()

if __name__ == '__main__':
    main()