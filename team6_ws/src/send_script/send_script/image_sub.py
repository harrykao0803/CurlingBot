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

sys.path.append('/home/robotics/workspace2/team6_ws/src/send_script/send_script/')
from utils import *

def obtain_anchor(img):
    blurred_image = cv2.GaussianBlur(img, (5, 5), 0)
    _, binary_image = cv2.threshold(blurred_image, 128, 255, cv2.THRESH_BINARY)
    binary_image = cv2.erode(binary_image, None, iterations=2)
    binary_image = cv2.dilate(binary_image, None, iterations=2)
    cv2.imwrite("AnchorMask.jpg", binary_image)
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result = []
    for contour in contours:
        M = cv2.moments(contour)
        if M["m00"] < 100: continue
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        result.append(cx)
        result.append(cy)
    if len(result) != 4:
        return None, None
    angle = np.arctan2(result[1] - result[3], result[0] - result[2])
    if result[0] < result[2]: return (result[0], result[1]), angle
    return (result[2], result[3]), angle  # return the position of one anchor (result[2], result[3]) and the angle between two anchors (angle)

def obtain_slide(img):
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filled_mask_green = np.zeros_like(img)
    for contour in contours:
        """ Trapezoid """
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)  # Get the four corners of the rectangle
        box = np.int0(box)         # Convert to integer
        cv2.fillPoly(filled_mask_green, [box], 255)
        """ Polygon """
        # epsilon = 0.02 * cv2.arcLength(contour, True)  # Adjust epsilon for precision
        # approx = cv2.approxPolyDP(contour, epsilon, True)
        # cv2.drawContours(filled_mask_green, [approx], -1, 255, thickness=cv2.FILLED)
    contours, _ = cv2.findContours(filled_mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.imwrite("SlideMask.jpg", filled_mask_green)

    # cv2.imshow("slide image", img)
    # cv2.imshow("slide image filled", filled_mask_green)
    # cv2.waitKey(0)                               # 暫停並等待任意按鍵命令
    # cv2.destroyAllWindows()

    slide_pos, slide_angle = None, None
    max_area = 0
    for contour in contours:
        M = cv2.moments(contour)
        if M["m00"] < 100: continue
        # Calculate centroid (cx, cy).
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        # Calculate orientation (principal angle).
        angle = 0.5 * np.arctan2(2 * M["mu11"], (M["mu20"] - M["mu02"]))
        angle_degrees = np.degrees(angle)
        if M["m00"] > max_area:
            max_area = M["m00"]
            slide_pos = (cx, cy)
            slide_angle = angle_degrees
    return slide_pos, slide_angle

def process(src_img):
    # preprocess the image
    contrast = 50
    brightness = -75
    src_img = src_img * (contrast / 127 + 1) - contrast + brightness
    src_img = np.clip(src_img, 0, 255)
    src_img = np.uint8(src_img)


    # convert the input image into grayscale
    src_image = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)
    if src_image is None:
        sys.exit("Could not read the image.")
    hsv_image = cv2.cvtColor(src_img, cv2.COLOR_BGR2HSV)

    # Thresholds
    lower_blue = np.array([105, 180, 100])
    upper_blue = np.array([125, 255, 255])
    lower_green = np.array([40, 70, 50])
    upper_green = np.array([80, 255, 255])
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])

    # Green mask
    mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
    slide_pos, slide_angle = obtain_slide(mask_green)
    while slide_pos == None:
        trigger_small_talk('Generate a speech to say when you cannot find your slide. Return the speech only')
        revert()
        print('Error: Slide is not found... Please put slide in the scene!')
        return None

    # Red mask
    mask_red = cv2.inRange(hsv_image, lower_red, upper_red)
    anchor_pos, anchor_angle = obtain_anchor(mask_red)
    print(anchor_pos)
    while anchor_pos == None:
        trigger_small_talk('Generate a speech to say when you cannot find the anchor to the house. Return the speech only')
        revert()
        print('Error: Red anchors are not detected. Please remove the objects above the red markers...')
        return None

    # Show the overlay of anchor on the original color image
    color_image = cv2.cvtColor(src_image, cv2.COLOR_GRAY2BGR)
    dx = int(1000 * np.cos(anchor_angle))
    dy = int(1000 * np.sin(anchor_angle))
    cv2.line(color_image, (int(anchor_pos[0]) - dx, int(anchor_pos[1]) - dy), (int(anchor_pos[0]) + dx, int(anchor_pos[1]) + dy), (255, 0, 0), 2)
    cv2.imwrite("AnchorOverlay.jpg", color_image)


    # Blue mask
    mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    closed_mask = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(closed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filled_mask = np.zeros_like(mask_blue)
    cv2.drawContours(filled_mask, contours, -1, 255, thickness=cv2.FILLED)



    # Preprocess the image to binary.
    blurred_image = cv2.GaussianBlur(filled_mask, (5, 5), 0)
    # cv2.imwrite("BlurredImage.jpg",blurred_image)
    _, binary_image = cv2.threshold(blurred_image, 128, 255, cv2.THRESH_BINARY)

    # Perform morphological opening to reduce noise.
    binary_image = cv2.erode(binary_image, None, iterations=2)
    binary_image = cv2.dilate(binary_image, None, iterations=2)

    # Find contours in the binary image.
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filled_image = np.zeros_like(binary_image)
    cv2.drawContours(filled_image, contours, -1, 255, thickness=cv2.FILLED)
    contours, _ = cv2.findContours(filled_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cv2.imwrite("ObjectMask.jpg", filled_image)

    # cv2.imshow("final image", filled_image)
    # cv2.waitKey(0)                               # 暫停並等待任意按鍵命令
    # cv2.destroyAllWindows()

    # Load the original image in color to display the results.
    color_image = cv2.cvtColor(src_image, cv2.COLOR_GRAY2BGR)
    result = []
    max_area, max_area_index = 0, -1
    for contour in contours:
        # Calculate moments to find the centroid and orientation.
        M = cv2.moments(contour)
        if M["m00"] < 200:  # Skip if the contour area is zero.
            continue

        if M["m00"] > max_area:
            max_area = M["m00"]
            max_area_index = len(result)

        # Calculate centroid (cx, cy).
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]

        # Calculate orientation (principal angle).
        angle = 0.5 * np.arctan2(2 * M["mu11"], (M["mu20"] - M["mu02"]))
        # angle = np.arctan2(xmax[1] - ymax[1], xmax[0] - ymax[0])
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

    if max_area_index != -1 and len(result) > 0:
        for i in range(3):
            result[i], result[max_area_index + i] = result[max_area_index + i], result[i]

    """
    Return format
    0 ~ 2: anchor position + angle
    3 ~ 5: slide position + angle
    6 ~ n: pin positions + angles
    """
    if anchor_pos == None:
        print("Anchor is not detected.")
    if slide_pos == None:
        print("Slide is not detected.")

    result = [anchor_pos[0], anchor_pos[1], anchor_angle] + [slide_pos[0], slide_pos[1], slide_angle] + result

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

        if result == None:
            # targetCam = "230.00, 230, 730, -180.00, 0.0, 135.00"
            # scriptCam = "PTP(\"CPP\"," + targetCam + ",100,200,0,false)"
            # send_script(scriptCam)
            # send_script("Vision_DoJob(job1)")
            pass
        else:
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