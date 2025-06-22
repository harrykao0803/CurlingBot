import cv2
import sys
import numpy as np
from mask_curling import get_stone_position

def obtain_anchor(img):
    blurred_image = cv2.GaussianBlur(img, (5, 5), 0)
    _, binary_image = cv2.threshold(blurred_image, 128, 255, cv2.THRESH_BINARY)
    binary_image = cv2.erode(binary_image, None, iterations=2)
    binary_image = cv2.dilate(binary_image, None, iterations=2)

    _, contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    result = []
    for contour in contours:
        M = cv2.moments(contour)
        if M["m00"] < 100: continue
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        result.append(cx)
        result.append(cy)

    if len(result) != 2:
        print("contours length:", len(contours))
        print("result len:", len(result))
        print('Warning: Detection does not function properly')
        return None, None

    return result
    # angle = np.arctan2(result[1] - result[3], result[0] - result[2])
    # if result[0] < result[2]: return (result[2], result[3], result[0], result[1]), angle
    # return (result[0], result[1], result[2], result[3]), angle  # return the position of one anchor (result[2], result[3]) and the angle between two anchors (angle)


# Load the image
image = cv2.imread('webcam_images/no_stone.jpg')

# ------------------------------------------ Green Spot ------------------------------------------

# Adjust contrast and brightness
contrast = 50
brightness = -75
# Ensure the image is in float32 to avoid overflow
src_img = image.astype(np.float32) * (contrast / 127 + 1) - contrast + brightness
src_img = np.clip(src_img, 0, 255)  # Clip the values to [0, 255]
src_img = np.uint8(src_img)

# Convert the input image to grayscale
src_image = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)
hsv_image = cv2.cvtColor(src_img, cv2.COLOR_BGR2HSV)


# Define the HSV range for detecting green spots (adjust these values as needed)
lower_light_green = np.array([30, 60, 60])  # Lower bound of the green color in HSV
upper_light_green = np.array([56, 255, 255])  # Upper bound of the green color in HSV

# Create a mask for the green color (light green in this case)
mask_light_green = cv2.inRange(hsv_image, lower_light_green, upper_light_green)
# cv2.imwrite("LightGreen Mask.jpg", mask_light_green)
green_anchor_pos = obtain_anchor(mask_light_green)

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
lower_yellow = np.array([20, 100, 100])  # Lower bound of the yellow color in HSV
upper_yellow = np.array([30, 255, 255])  # Upper bound of the yellow color in HSV

# Create a mask for the yellow color (yellow in this case)
mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
# cv2.imwrite("Yellow Mask.jpg", mask_yellow)
yellow_anchor_pos = obtain_anchor(mask_yellow)

# # testing
# cv2.circle(image, (int(yellow_anchor_pos[0]), int(yellow_anchor_pos[1])), 5, (0, 255, 255), -1) # Bottom yellow spot (If capture from the left of the target)



# ------------------------------------------ Purple Spot ------------------------------------------

# Define the HSV range for detecting purple spots (adjust these values as needed)
lower_purple = np.array([125, 0, 25])  # Lower bound of the purple color in HSV
upper_purple = np.array([170, 119, 255])  # Upper bound of the purple color in HSV

# Create a mask for the purple color (purple in this case)
mask_purple = cv2.inRange(image, lower_purple, upper_purple)
# cv2.imwrite("Purple Mask.jpg", mask_purple)
purple_anchor_pos = obtain_anchor(mask_purple)

# # testing
# cv2.circle(image, (int(purple_anchor_pos[0]), int(purple_anchor_pos[1])), 5, (255, 0, 255), -1) # Bottom purple spot (If capture from the left of the target)



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

result_Blue, result_Orange = get_stone_position('webcam_images/second_round.jpg')

Blue_Stone_pos = []
for i in range(0, len(result_Blue), 2):
    converted_Blue_stone_pos = np.dot(rotation_matrix, [result_Blue[i] - x_offset, result_Blue[i+1] - y_offset])
    Blue_Stone_pos.append(converted_Blue_stone_pos)

Orange_Stone_pos = []
for i in range(0, len(result_Orange), 2):
    converted_Orange_stone_pos = np.dot(rotation_matrix, [result_Orange[i] - x_offset, result_Orange[i+1] - y_offset])
    Orange_Stone_pos.append(converted_Orange_stone_pos)

print(Blue_Stone_pos)
print(Orange_Stone_pos)


# ------------------------------------------ Save Image Fuck u ------------------------------------------

for i in range(100):
    cv2.circle(image, (i, 0), 5, (0, 0, 255), -1)

for i in range(100):
    cv2.circle(image, (0, i), 5, (255, 255, 255), -1)

# If you need to save the output image
success = cv2.imwrite("Detect_XY_Axis.jpg", image)
if not success:
    print("Error saving the image")





