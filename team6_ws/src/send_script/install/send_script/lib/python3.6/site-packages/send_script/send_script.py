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




class BlockSub(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(Float32MultiArray, '/techman_block', self.block_callback, 10)
        self.subscription

    def block_callback(self, data):

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
        for i in range(1, 9, 3):
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
        for i in range(0, 9, 3):
            mat = np.array([[data[i]], [data[i + 1]], [1]])  # Making mat a 3x1 column vector

            # Multiply the transformation matrix with the vector
            point = np.dot(rotation_matrix, mat)

            point[0] += offset_x
            point[1] += offset_y

            result.append(point[0][0])
            result.append(point[1][0])
            result.append(-1 * data[i + 2] + 45)

        print(result)

        # Do something to get three cubes' position, the below list is just for example. (TO BE DONE...)
        block_positions = [
            f"{result[0]}, {result[1]}, 115, -180.00, 0.0, {result[2]}",
            f"{result[3]}, {result[4]}, 115, -180.00, 0.0, {result[5]}",
            f"{result[6]}, {result[7]}, 115, -180.00, 0.0, {result[8]}",
        ]

        block_positions_mid = [
            f"{result[0]}, {result[1]}, 200, -180.00, 0.0, {result[2]}",
            f"{result[3]}, {result[4]}, 200, -180.00, 0.0, {result[5]}",
            f"{result[6]}, {result[7]}, 200, -180.00, 0.0, {result[8]}",
        ]

        # places to stack the three cubes, the three elements in the list should have difference only in z. (寫死)
        target_positions_mid = [
            "730.00, 200, 200, -180.00, 0.0, 135.00",
            "730.00, 200, 200, -180.00, 0.0, 135.00",
            "730.00, 200, 200, -180.00, 0.0, 135.00"
        ]

        target_positions = [
            "730.00, 200, 120, -180.00, 0.0, 135.00",
            "730.00, 200, 150, -180.00, 0.0, 135.00",
            "730.00, 200, 180, -180.00, 0.0, 135.00"
        ]




        sleep_time=8

        for i in range(3):

            print(f"Block {i} processing... cnm ><")

            block_position = block_positions[i]
            block_pos_script = "PTP(\"CPP\"," + block_position + ",100,200,0,false)"
            target_position = target_positions[i]
            target_pos_script = "PTP(\"CPP\"," + target_position + ",100,200,0,false)"


            block_position_mid = block_positions_mid[i]
            block_pos_script_mid = "PTP(\"CPP\"," + block_position_mid + ",100,200,0,false)"
            target_position_mid = target_positions_mid[i]
            target_pos_script_mid = "PTP(\"CPP\"," + target_position_mid + ",100,200,0,false)"



            # elevate block
            send_script(block_pos_script_mid)
            #time.sleep(sleep_time)

            # move gripper to block's position
            send_script(block_pos_script)
            time.sleep(sleep_time)


            # close gripper
            set_io(1.0)
            time.sleep(sleep_time)


            # elevate block
            send_script(block_pos_script_mid)
            #time.sleep(sleep_time)


            # transfer block
            send_script(target_pos_script_mid)
            #time.sleep(sleep_time)



            # move gripper to the place we're going to stack the blocks
            send_script(target_pos_script)
            time.sleep(sleep_time)


            # open gripper
            set_io(0.0)
            time.sleep(sleep_time)



            # elevate block
            send_script(target_pos_script_mid)
            #time.sleep(sleep_time)






            print(f"Block {i} processing... end ><")

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
    targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
    script1 = "PTP(\"CPP\"," + targetP1 + ",100,200,0,false)"


# What does Vision_DoJob do? Try to use it...   (My opinion: take a picture)
# -------------------------------------------------

    print("Start moving nmsl :)")




    # move camera to the initial position and take a picture.
    send_script(script1)
    send_script("Vision_DoJob(job1)")
    cv2.waitKey(1)

    print("Picture taken mdfk :(")

    node = BlockSub("block_sub")

    rclpy.spin(node)

    # # Do something to get three cubes' position, the below list is just for example. (TO BE DONE...)
    # block_positions = [
    #     "250.00, 250, 500, -180.00, 0.0, 135.00",
    #     "250.00, 250, 500, -180.00, 0.0, 135.00",
    #     "250.00, 250, 500, -180.00, 0.0, 135.00",
    # ]

    # # places to stack the three cubes, the three elements in the list should have difference only in z. (寫死)
    # target_positions = [
    #     "530.00, 200, 100, -180.00, 0.0, 135.00",
    #     "530.00, 200, 130, -180.00, 0.0, 135.00",
    #     "530.00, 200, 160, -180.00, 0.0, 135.00"
    # ]

    # for i in range(3):

    #     print(f"Block {i} processing... cnm ><")

    #     block_position = block_positions[i]
    #     block_pos_script = "PTP(\"CPP\"," + block_position + ",100,200,0,false)"
    #     target_position = target_positions[i]
    #     target_pos_script = "PTP(\"CPP\"," + target_position + ",100,200,0,false)"

    #     # move gripper to block's position
    #     send_script(block_pos_script)
    #     time.sleep(1.0)

    #     # close gripper
    #     set_io(1.0)
    #     time.sleep(1.0)

    #     # move gripper to the place we're going to stack the blocks
    #     send_script(target_pos_script)
    #     time.sleep(1.0)

    #     # open gripper
    #     set_io(0.0)
    #     time.sleep(1.0)

    # send_script(script1)

#--------------------------------------------------

    rclpy.shutdown()


if __name__ == '__main__':
    main()





