#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import time
import sys
import cv2
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *
from std_msgs.msg import Float32MultiArray
import numpy as np
import json
sys.path.append('/home/robotics/workspace2/team6_ws/src/send_script/send_script/')
from utils import *

class BlockSub(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(Float32MultiArray, '/techman_block', self.block_callback, 10)
        self.subscription

    def block_callback(self, data):
        short_sleep_time=2
        long_sleep_time=5.5

        # Load trajectory selection
        with open('command.jsonl', 'r') as file:
            commands = json.load(file)
        command = commands[-1] # Fetch the latest command
        cmd_angle = command['angle']
        cmd_voffset = command['v_offset']
        cmd_hoffset = command['h_offset']
        cmd_strength = command['strength']
        print(command)

        data = data.data
        print("Callback data: ", data)

        self.get_logger().info('Received block position and angle')

        # TODO (write your code here)

        # initial camera position
        targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
        script1 = "PTP(\"CPP\"," + targetP1 + ",100,200,0,false)"

        # Transform data
        scale = 0.4879
        for i in range(0, len(data), 3):
            data[i] *= scale
            data[i + 1] *= scale


        # Negate along x aixs (y *= -1)
        for i in range(1, len(data), 3):
            data[i] *= -1

        angle = np.radians(-45)

        # Target position
        offset_x = 256
        offset_y = 680

        # Transformation matrix (3x3) -> rotates about z axis by 45 degrees
        rotation_matrix = np.array([
            [np.cos(angle), -1 * np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1]
        ])

        result = []
        # Matrix `mat` (2D column vector with 3 elements)
        for i in range(0, len(data), 3):
            mat = np.array([[data[i]], [data[i + 1]], [1]])  # Making mat a 3x1 column vector

            # Multiply the transformation matrix with the vector
            point = np.dot(rotation_matrix, mat)

            point[0] += offset_x
            point[1] += offset_y

            result.append(point[0][0])
            result.append(point[1][0])
            result.append(-1 * data[i + 2] + 135)

        print(result)

        # Divide the data into anchor data and object data
        anchor_info = result[0: 3]
        slide_info = result[3: 6]
        block_info = result[6:]
        print(len(block_info) / 3, 'objects detected')

        if len(block_info) == 0:
            # No block identified
            trigger_small_talk('Generate a speech to say when you cannot find your curling stones. Return the speech only')
            # trigger_speech('Hmmmm... Does any one see my stone? I need someone to help me pick up a blue stone. pleaseee')
            revert()
            return

        """ Command offset """
        anchor_angle = data[2] - 180
        cmd_offset = (-cmd_voffset * np.cos(np.radians(anchor_angle)) + cmd_hoffset * np.cos(np.radians(anchor_angle + 90)), -cmd_voffset * np.sin(np.radians(anchor_angle)) + cmd_hoffset * np.sin(np.radians(anchor_angle + 90)))
        anchor_angle += cmd_angle

        """ Move Slide to the intended place """
        # Slide offset compensation
        slide_length = 100
        slide_offset = (slide_length * np.cos(np.radians(slide_info[2])), slide_length * np.sin(np.radians(slide_info[2])))
        slide_info[0] += slide_offset[0]
        slide_info[1] += slide_offset[1]
        slide_position = f"{slide_info[0]}, {slide_info[1]}, 240, -180.00, 0.0, {slide_info[2]}"
        slide_position_mid = f"{slide_info[0]}, {slide_info[1]}, 400, -180.00, 0.0, {slide_info[2]}"

        # Slide target positions
        anchor_length = 220
        anchor_offset = (anchor_length * np.cos(np.radians(data[2] - 180)), anchor_length * np.sin(np.radians(data[2] - 180)))
        anchor_pos = (anchor_info[0] + anchor_offset[0] + cmd_offset[0], anchor_info[1] + anchor_offset[1] + cmd_offset[1])
        slide_target_position_mid = f"{anchor_pos[0]}, {anchor_pos[1]}, 400, -180.00, 0.0, {anchor_angle}"
        slide_target_position = f"{anchor_pos[0]}, {anchor_pos[1]}, 255, -180.00, 0.0, {anchor_angle}"

        # Adjust slide position
        send_script(f"PTP(\"CPP\",{slide_position_mid},100,200,0,false)")
        send_script(f"PTP(\"CPP\",{slide_position},100,200,0,false)")
        time.sleep(long_sleep_time)
        set_io(1.0)
        time.sleep(short_sleep_time)
        send_script(f"PTP(\"CPP\",{slide_position_mid},100,200,0,false)")
        send_script(f"PTP(\"CPP\",{slide_target_position_mid},100,200,0,false)")
        send_script(f"PTP(\"CPP\",{slide_target_position_mid},100,200,0,false)")
        send_script(f"PTP(\"CPP\",{slide_target_position},100,200,0,false)")
        time.sleep(short_sleep_time)
        set_io(0.0)
        time.sleep(short_sleep_time)
        send_script(f"PTP(\"CPP\",{slide_target_position_mid},100,200,0,false)")


        """ Move block positions """
        block_positions, block_positions_mid = [], []
        for i in range(0, len(block_info), 3):
            block_positions.append(f"{block_info[i]}, {block_info[i + 1]}, 105, -180.00, 0.0, {block_info[i + 2]}")
            block_positions_mid.append(f"{block_info[i]}, {block_info[i + 1]}, 180, -180.00, 0.0, {block_info[i + 2]}")
        push_strength = cmd_strength # Max to 210 (Not recommended)
        push_offset = ((210 - push_strength) * np.cos(np.radians(anchor_angle + 180)), (210 - push_strength) * np.sin(np.radians(anchor_angle + 180)), push_strength * 0.59)
        target_positions_mid = [f"{anchor_pos[0] + push_offset[0]}, {anchor_pos[1] + push_offset[1]}, 300, -180.00, 0.0, {anchor_angle}" for _ in range(int(len(result) / 3))]
        target_positions = [f"{anchor_pos[0] + push_offset[0]}, {anchor_pos[1] + push_offset[1]}, {145 + push_offset[2]}, -180.00, 0.0, {anchor_angle}" for _ in range(int(len(result) / 3))]

        # Move blocks
        for i in range(int(len(block_positions))):
            print(f"Block {i} processing... cnm ><")

            block_position = block_positions[i]
            block_pos_script = "PTP(\"CPP\"," + block_position + ",100,200,0,false)"
            block_position_mid = block_positions_mid[i]
            block_pos_script_mid = "PTP(\"CPP\"," + block_position_mid + ",100,200,0,false)"


            target_position_mid = target_positions_mid[i]
            target_pos_script_mid = "PTP(\"CPP\"," + target_position_mid + ",100,200,0,false)"
            target_position = target_positions[i]
            target_pos_script = "PTP(\"CPP\"," + target_position + ",100,200,0,false)"


            # elevate block
            send_script(block_pos_script_mid)
            #time.sleep(sleep_time)

            # move gripper to block's position
            send_script(block_pos_script)
            time.sleep(long_sleep_time)

            # close gripper
            set_io(1.0)
            time.sleep(short_sleep_time)

            # elevate block
            send_script(block_pos_script_mid)
            #time.sleep(sleep_time)

            send_script(target_pos_script_mid)
            send_script(target_pos_script)

            # open gripper
            set_io(0.0)
            time.sleep(short_sleep_time)

            print(f"Block {i} processing... end ><")

            # original_target = "230.00, 230, 730, -180.00, 0.0, 135.00"
            # original_script = "PTP(\"CPP\"," + original_target + ",100,200,0,false)"
            # send_script(original_script)

            break # So that it only process once

        send_script(script1)


# arm client: send script to robot arm in order to control it.
# script = "PTP("CPP", x, y, z, a, b, c, ...)" => move camera to the specified position
# script = "Vision_DoJob(job1)" => execute the vision task named 'job1'
def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()


# gripper client: control the opening and closing of the gripper
# state = 1.0: close gripper, state = 0.0: open gripper
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

    rclpy.init(args=args)

    #--- move command by joint angle ---#
    # script = 'PTP(\"CPP\",45,0,90,0,90,0,35,200,0,false)'

    #--- move command by end effector's pose (x,y,z,a,b,c) ---#
    # targetP1 = "398.97, -122.27, 748.26, -179.62, 0.25, 90.12"s

    # Initial camera position for taking image (Please do not change the values)
    # For right arm: targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
    # For left  arm: targetP1 = "350.00, 350, 730, -180.00, 0.0, 135.00"

    # initial camera position
    # targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
    # script1 = "PTP(\"CPP\"," + targetP1 + ",100,200,0,false)"

# What does Vision_DoJob do? Try to use it...   (My opinion: take a picture)
# -------------------------------------------------

    # print("Start moving nmslcnm :)")
    # set_io(0.0)


    # # move camera to the initial position and take a picture.
    # send_script(script1)
    # send_script("Vision_DoJob(job1)")
    # cv2.waitKey(1)

    # print("Curling picture taken mdfk :P")

    node2 = BlockSub("block_sub")
    rclpy.spin(node2)

    # 關閉 node (block_sub)
    node2.destroy_node()


#--------------------------------------------------

    rclpy.shutdown()


if __name__ == '__main__':
    main()





