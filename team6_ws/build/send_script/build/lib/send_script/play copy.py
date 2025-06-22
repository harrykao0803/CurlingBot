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
sys.path.append('/home/robotics/workspace2/team6_ws/src/send_script/send_script/')
from utils import *
import json
import random

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

    # 初始化 ROS
    rclpy.init(args=args)

    print("Game start :)")
    targetCam = "230.00, 230, 730, -180.00, 0.0, 135.00"
    scriptCam = "PTP(\"CPP\"," + targetCam + ",100,200,0,false)"
    set_io(0.0)

    trajectory_plan = []
    with open('command.jsonl', 'w') as file:
        json.dump(trajectory_plan, file)

    calibration_img = take_picture()
    cv2.imwrite('Calibration_image.jpg', calibration_img)
    webcam_params = get_webcam_parameters(calibration_img)
    print(webcam_params)


    trigger_small_talk('Say something to welcome the player to play mini-curling')
    # Run throw rock for 4 times with pauses

    while(1):
        res=get_state()
        state,round= res['state'],res['round']
        if state == 'robot':

    num_throws = 1
    for round in range(1,num_throws+1):
        print(f'Round: {round}')

        webcam_img = take_picture()
        blue_pos, orange_pos = get_stone_position_calibrated(webcam_img, webcam_params)
        print(f"blue_pos: {blue_pos}, orange_pos: {orange_pos}")
        #trajectory_selection = random.randint(0, 9)
        #trajectory_selection = 1
        trajectory = obtain_trajectory_params(9)
        trajectory['index'] = round
        trajectory_plan.append(trajectory)
        with open('command.jsonl', 'w') as file:
            json.dump(trajectory_plan, file)

        send_script(scriptCam)
        send_script("Vision_DoJob(job1)")

        # User turn
        trigger_small_talk('Tell player it is his turn to throw curling...')
        # User fininsh
        input("Press Enter to continue to the next move...")



    # Compute the score
    """ TODO: Invoke webcam for score check """
    webcam_img = take_picture()
    result = calculate_score(webcam_img, webcam_params)
    # print(result)
    if len(result) == 0:
        print('No rock on the plane...')
    else:
        score = len(result)
        print(result[0][0], 'won the game! with', score, 'points')

    """  """



    # 關閉 ROS
    rclpy.shutdown()



if __name__ == '__main__':
    main()





