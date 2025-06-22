import cv2
import numpy as np
import requests


'''
Constants: List of trajectories!
'''
def obtain_trajectory_params(index):
    x=15
    params = {

        0: { 'angle': 0.0, 'v_offset': 0.0, 'h_offset': 0.0, 'strength': 121.0+x },
        1: { 'angle': 3.0, 'v_offset': 5.0, 'h_offset': 50.0, 'strength': 119+x },
        2: { 'angle': -3.0, 'v_offset': 10.0, 'h_offset': -70.0, 'strength':131.0+x },
        3: { 'angle': -12.0, 'v_offset': 10.0, 'h_offset': -200.0, 'strength': 125.0+x },
        4: { 'angle': 6.0, 'v_offset': 100.0, 'h_offset': 120.0, 'strength': 100.0+x },
        5: { 'angle': -5.0, 'v_offset': 15.0, 'h_offset': -90.0, 'strength': 143.0+x },
        6: { 'angle': 0.0, 'v_offset': -5.0, 'h_offset': 60.0, 'strength': 141.0+x },
        #7: { 'angle': 5.0, 'v_offset': 70.0, 'h_offset': 80.0, 'strength': 115.0+x },
        7: { 'angle': -6.0, 'v_offset': 2.0, 'h_offset': -50.0, 'strength': 120.0+x },
        8: { 'angle': -6.0, 'v_offset': 10.0, 'h_offset': -90.0, 'strength': 125.0+x },

    }

    return params[index]


base_url = 'https://tomcheng.me'
def trigger_small_talk(prompt, instruction = ''):
    requests.post(base_url+"/trigger_small_talk",json={'prompt': prompt, 'instruction': instruction}, headers={'Content-Type': 'application/json'})

def trigger_speech(text):
    requests.post(base_url+"/trigger_speech",json={'speech': text }, headers={'Content-Type': 'application/json'})

def revert():
    requests.post(base_url+"/revert",json={}, headers={'Content-Type': 'application/json'})


def encode_result(result):
    result_strs = []
    for r in result:
        r = [ str(i) for i in r ]
        result_strs.append(' '.join(r))
    result_str = ' '.join(result_strs)
    return result_str

def set_result(res):
    result_str = encode_result(res)
    print(result_str)
    requests.post(base_url+"/set_result",data={'result_pos': result_str})

def get_result():
    response=requests.get(base_url+"/get_result")
    return response.json()

def get_state():
    response=requests.get(base_url+"/get_state")
    return response.json()

def progress():
    requests.post(base_url+"/progress",json={},headers={'Content-Type': 'application/json'})

def take_picture():
    # Initialize the webcam (0 is the default camera)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not access the camera.")
        exit()

    # Capture a frame
    ret, frame = cap.read()

    # Release the camera
    cap.release()

    if ret:
        return frame
    else:
        print('Error: Unable to take picture')
        return None


def resolve_contours(res):
    if len(res) == 2:
        contours, _ = res
    elif len(res) == 3:
        _, contours, _ = res
    return contours


def get_stone_position(src_img):  # testing image_directory: 'webcam_images/second_round.jpg'

    # ---------------------------------------------- Detect Blue stones ------------------------------------------------------------

    image = src_img

    # Blue
    # lower = np.array([96, 156, 100])
    # upper = np.array([135, 255, 220])
    lower = np.array([95, 100, 100])  # Slightly lower thresholds for hue, saturation, and brightness
    upper = np.array([130, 255, 255])

    mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), lower, upper)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    closed_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours = resolve_contours(cv2.findContours(closed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))
    filled_mask = np.zeros_like(mask)
    cv2.drawContours(filled_mask, contours, -1, 255, thickness=cv2.FILLED)

    # Preprocess the image to binary.
    blurred_image = cv2.GaussianBlur(filled_mask, (5, 5), 0)

    # cv2.imwrite("BlurredImage.jpg",blurred_image)
    _, binary_image = cv2.threshold(blurred_image, 128, 255, cv2.THRESH_BINARY)

    # Perform morphological opening to reduce noise.
    binary_image = cv2.erode(binary_image, None, iterations=2)
    binary_image = cv2.dilate(binary_image, None, iterations=2)

    # Find contours in the binary image.
    contours = resolve_contours(cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))
    filled_image = np.zeros_like(binary_image)
    cv2.drawContours(filled_image, contours, -1, 255, thickness=cv2.FILLED)
    contours = resolve_contours(cv2.findContours(filled_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))

    # output = cv2.bitwise_and(image, image, mask = filled_mask)
    # cv2.imwrite("output.jpg", output)
    # cv2.imwrite("mask.jpg", filled_mask)

    result_Blue = []
    color_image = image
    for contour in contours:
        # Calculate moments to find the centroid and orientation.
        M = cv2.moments(contour)
        if M["m00"] < 1000:  # Skip if the contour area is zero.
            continue

        if M["m00"] > 3000:  # Skip if the contour area is zero.
            continue

        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]

        # # Calculate orientation (principal angle).
        # angle = 0.5 * np.arctan2(2 * M["mu11"], (M["mu20"] - M["mu02"]))
        # # angle = np.arctan2(xmax[1] - ymax[1], xmax[0] - ymax[0])
        # angle_degrees = np.degrees(angle)

        # Draw the centroid on the image.
        # cv2.circle(color_image, (int(cx), int(cy)), 5, (0, 255, 0), -1)

        # # Draw the principal direction as a bidirectional line.
        # length = 400  # Length of the line in one direction
        # dx = int(length * np.cos(angle))
        # dy = int(length * np.sin(angle))

        # #  Draw the line extending equally in both directions from the centroid
        # cv2.line(color_image, (int(cx) - dx, int(cy) - dy), (int(cx) + dx, int(cy) + dy), (255, 0, 0), 2)

        # # Display the centroid coordinates and principal angle.
        # print(f"Centroid: ({cx}, {cy}), Principal Angle: {angle_degrees:.2f} degrees")

        result_Blue.append(cx)
        result_Blue.append(cy)

    success = cv2.imwrite("Object_Detection_Blue.jpg", filled_image)


    # ---------------------------------------------- Detect Orange stones ------------------------------------------------------------

    image = src_img

    # Orange
    # lower = np.array([0, 100, 210])
    # upper = np.array([43, 255, 255])
    lower = np.array([8, 150, 200])  # Adjusted lower bound for orange
    upper = np.array([20, 255, 255])

    mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), lower, upper)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    closed_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours = resolve_contours(cv2.findContours(closed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))
    filled_mask = np.zeros_like(mask)
    cv2.drawContours(filled_mask, contours, -1, 255, thickness=cv2.FILLED)

    # Preprocess the image to binary.
    blurred_image = cv2.GaussianBlur(filled_mask, (5, 5), 0)

    # cv2.imwrite("BlurredImage.jpg",blurred_image)
    _, binary_image = cv2.threshold(blurred_image, 128, 255, cv2.THRESH_BINARY)

    # Perform morphological opening to reduce noise.
    binary_image = cv2.erode(binary_image, None, iterations=2)
    binary_image = cv2.dilate(binary_image, None, iterations=2)

    # Find contours in the binary image.
    contours = resolve_contours(cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))
    filled_image = np.zeros_like(binary_image)
    cv2.drawContours(filled_image, contours, -1, 255, thickness=cv2.FILLED)
    contours = resolve_contours(cv2.findContours(filled_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))

    result_Orange = []
    color_image = image
    for contour in contours:
        # Calculate moments to find the centroid and orientation.
        M = cv2.moments(contour)
        if M["m00"] < 1000:  # Skip if the contour area is zero.
            continue

        if M["m00"] > 3000:  # Skip if the contour area is zero.
            continue

        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]

        # # Calculate orientation (principal angle).
        # angle = 0.5 * np.arctan2(2 * M["mu11"], (M["mu20"] - M["mu02"]))
        # # angle = np.arctan2(xmax[1] - ymax[1], xmax[0] - ymax[0])
        # angle_degrees = np.degrees(angle)

        # # Draw the centroid on the image.
        # cv2.circle(color_image, (int(cx), int(cy)), 5, (0, 255, 0), -1)

        # # Draw the principal direction as a bidirectional line.
        # length = 400  # Length of the line in one direction
        # dx = int(length * np.cos(angle))
        # dy = int(length * np.sin(angle))

        # # Draw the line extending equally in both directions from the centroid
        # cv2.line(color_image, (int(cx) - dx, int(cy) - dy), (int(cx) + dx, int(cy) + dy), (255, 0, 0), 2)

        # # Display the centroid coordinates and principal angle.
        # print(f"Centroid: ({cx}, {cy}), Principal Angle: {angle_degrees:.2f} degrees")

        result_Orange.append(cx)
        result_Orange.append(cy)

    success = cv2.imwrite("Object_Detection_Orange.jpg", filled_image)

    return result_Blue, result_Orange






def get_webcam_parameters(src_img_raw):

    def obtain_anchor(img):
        blurred_image = cv2.GaussianBlur(img, (5, 5), 0)
        _, binary_image = cv2.threshold(blurred_image, 128, 255, cv2.THRESH_BINARY)
        binary_image = cv2.erode(binary_image, None, iterations=2)
        binary_image = cv2.dilate(binary_image, None, iterations=2)

        contours = resolve_contours(cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))


        result = []
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] < 100: continue
            if M["m00"] > 2000: continue
            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]
            result.append(cx)
            result.append(cy)

        if len(result) != 2:
            # print("contours length:", len(contours))
            # print("result len:", len(result))
            # print('Warning: Detection does not function properly')
            return None, None

        return result
        # angle = np.arctan2(result[1] - result[3], result[0] - result[2])
        # if result[0] < result[2]: return (result[2], result[3], result[0], result[1]), angle
        # return (result[0], result[1], result[2], result[3]), angle  # return the position of one anchor (result[2], result[3]) and the angle between two anchors (angle)


    # Load the image
    image = src_img_raw

    # ------------------------------------------ Green Spot (Switched to Red instead) ------------------------------------------

    # # Adjust contrast and brightness
    # contrast = 50
    # brightness = -75
    # # Ensure the image is in float32 to avoid overflow
    # src_img = image.astype(np.float32) * (contrast / 127 + 1) - contrast + brightness
    # src_img = np.clip(src_img, 0, 255)  # Clip the values to [0, 255]
    # src_img = np.uint8(src_img)

    # # Convert the input image to grayscale
    # src_image = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)
    # hsv_image = cv2.cvtColor(src_img, cv2.COLOR_BGR2HSV)


    # # Define the HSV range for detecting green spots (adjust these values as needed)
    # lower_light_green = np.array([30, 60, 60])  # Lower bound of the green color in HSV
    # upper_light_green = np.array([56, 255, 255])  # Upper bound of the green color in HSV

    # # Create a mask for the green color (light green in this case)
    # mask_light_green = cv2.inRange(hsv_image, lower_light_green, upper_light_green)
    # cv2.imwrite("LightGreen Mask.jpg", mask_light_green)
    # green_anchor_pos = obtain_anchor(mask_light_green)


    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    src_img = image
    hsv_image = cv2.cvtColor(src_img, cv2.COLOR_BGR2HSV)
    mask_red = cv2.inRange(hsv_image, lower_red, upper_red)
    cv2.imwrite("Red Mask.jpg", mask_red)
    red_anchor_pos = obtain_anchor(mask_red)


    # # testing
    # cv2.circle(image, (int(green_anchor_pos[0]), int(green_anchor_pos[1])), 5, (0, 255, 0), -1) # Bottom green spot (If capture from the left of the target)


    # ------------------------------------------ Yellow Spot -----------------------------------------

    # Adjust contrast and brightness
    contrast = 10
    brightness = -75
    # Ensure the image is in float32 to avoid overflow
    src_img = image.astype(np.float32) * (contrast / 127 + 1) - contrast + brightness
    src_img = np.clip(src_img, 0, 255)  # Clip the values to [0, 255]
    src_img = np.uint8(src_img)

    # Convert the input image to grayscale
    src_image = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)
    hsv_image = cv2.cvtColor(src_img, cv2.COLOR_BGR2HSV)


    # Define the HSV range for detecting yellow spots (adjust these values as needed)
    # lower_yellow = np.array([20, 100, 100])  # Lower bound of the yellow color in HSV
    # upper_yellow = np.array([30, 255, 255])  # Upper bound of the yellow color in HSV
    lower_yellow = np.array([20, 50, 100])  # Lower bounds for yellow (including reflection)
    upper_yellow = np.array([40, 255, 255])


    # Create a mask for the yellow color (yellow in this case)
    mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
    cv2.imwrite("Yellow Mask.jpg", mask_yellow)
    yellow_anchor_pos = obtain_anchor(mask_yellow)

    # # testing
    # cv2.circle(image, (int(yellow_anchor_pos[0]), int(yellow_anchor_pos[1])), 5, (0, 255, 255), -1) # Bottom yellow spot (If capture from the left of the target)



    # ------------------------------------------ Purple Spot ------------------------------------------

    # Define the HSV range for detecting purple spots (adjust these values as needed)
    lower_purple = np.array([125, 0, 25])  # Lower bound of the purple color in HSV
    upper_purple = np.array([170, 119, 255])  # Upper bound of the purple color in HSV

    # Create a mask for the purple color (purple in this case)
    mask_purple = cv2.inRange(image, lower_purple, upper_purple)
    cv2.imwrite("Purple Mask.jpg", mask_purple)
    purple_anchor_pos = obtain_anchor(mask_purple)


    # # testing
    # cv2.circle(image, (int(purple_anchor_pos[0]), int(purple_anchor_pos[1])), 5, (255, 0, 255), -1) # Bottom purple spot (If capture from the left of the target)

    return (red_anchor_pos, yellow_anchor_pos, purple_anchor_pos)


def get_stone_position_calibrated(src_img_raw, params):
    green_anchor_pos, yellow_anchor_pos, purple_anchor_pos = params

    # ------------------------------------------ Frame conversion ------------------------------------------

    x_offset = yellow_anchor_pos[0]
    y_offset = yellow_anchor_pos[1]

    angle = (-1) * np.arctan2(green_anchor_pos[1] - yellow_anchor_pos[1], green_anchor_pos[0] - yellow_anchor_pos[0])  # radian

    rotation_matrix = np.array([
                [np.cos(angle), -1 * np.sin(angle)],
                [np.sin(angle), np.cos(angle)]
            ])

    # yellow_spot_position = np.dot(rotation_matrix, [yellow_anchor_pos[0] - x_offset, yellow_anchor_pos[1] - y_offset])
    # green_spot_position = np.dot(rotation_matrix, [green_anchor_pos[0] - x_offset, green_anchor_pos[1] - y_offset])
    # purple_spot_position = np.dot(rotation_matrix, [purple_anchor_pos[0] - x_offset, purple_anchor_pos[1] - y_offset])

    result_Blue, result_Orange = get_stone_position(src_img_raw)

    Blue_Stone_pos = []
    for i in range(0, len(result_Blue), 2):
        converted_Blue_stone_pos = np.dot(rotation_matrix, [result_Blue[i] - x_offset, result_Blue[i+1] - y_offset])
        Blue_Stone_pos.append(converted_Blue_stone_pos)

    Orange_Stone_pos = []
    for i in range(0, len(result_Orange), 2):
        converted_Orange_stone_pos = np.dot(rotation_matrix, [result_Orange[i] - x_offset, result_Orange[i+1] - y_offset])
        Orange_Stone_pos.append(converted_Orange_stone_pos)


    # ------------------------------------------ Save Image Fuck u ------------------------------------------

    for i in range(100):
        cv2.circle(src_img_raw, (i, 0), 5, (0, 0, 255), -1)

    for i in range(100):
        cv2.circle(src_img_raw, (0, i), 5, (255, 255, 255), -1)

    # If you need to save the output image
    success = cv2.imwrite("Detect_XY_Axis.jpg", src_img_raw)
    if not success:
        print("Error saving the image")


    return Blue_Stone_pos, Orange_Stone_pos

def distance(pos):
    return pos[0] * pos[0] + pos[1] * pos[1]

def calculate_score(src_img, webcam_params, exhaustive = False):
    blue_pos, orange_pos = get_stone_position_calibrated(src_img, webcam_params)
    if len(blue_pos) == 0 and len(orange_pos) == 0:
        print('Error: No rock detected in the map')
        return None

    blue_record = [ ('blue', pos[0], pos[1], distance(pos)) for pos in blue_pos ]
    orange_record = [ ('orange', pos[0], pos[1], distance(pos)) for pos in orange_pos ]
    records = blue_record + orange_record
    records = sorted(records, key=lambda x: x[3])
    # print("Original Records:", records)

    if exhaustive:
        return records

    for i in range(1, len(records)):
        if records[0][0] != records[i][0]:
            records = records[:i]
            break

    # print("Resulting Records:", records)
    return records