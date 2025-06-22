import cv2
import numpy as np


def get_stone_position(image_directory):  # testing image_directory: 'webcam_images/second_round.jpg'

    # ---------------------------------------------- Detect Blue stones ------------------------------------------------------------

    image = cv2.imread(image_directory)

    # Blue
    lower = np.array([96, 156, 100])
    upper = np.array([135, 255, 220])

    mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), lower, upper)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    closed_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    _, contours, _= cv2.findContours(closed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
    _, contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filled_image = np.zeros_like(binary_image)
    cv2.drawContours(filled_image, contours, -1, 255, thickness=cv2.FILLED)
    _, contours, _ = cv2.findContours(filled_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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

    # success = cv2.imwrite("Object_Detection_Blue.jpg", color_image)


    # ---------------------------------------------- Detect Orange stones ------------------------------------------------------------

    image = cv2.imread(image_directory)

    # Orange
    lower = np. array([0, 100, 210])
    upper = np.array([43, 255, 255])

    mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), lower, upper)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    closed_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    _, contours, _= cv2.findContours(closed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
    _, contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filled_image = np.zeros_like(binary_image)
    cv2.drawContours(filled_image, contours, -1, 255, thickness=cv2.FILLED)
    _, contours, _ = cv2.findContours(filled_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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

    # success = cv2.imwrite("Object_Detection_Orange.jpg", color_image)

    return result_Blue, result_Orange