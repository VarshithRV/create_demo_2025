import cv2
import numpy as np
import sys

# def detect_red_circle(image_path):
#     # Load the image
#     frame = cv2.imread(image_path)
#     if frame is None:
#         print("Error: Unable to load image.")
#         return
    
#     # Convert to HSV color space
#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
#     # Define range for red color (filled red color detection)
#     lower_red1 = np.array([0, 150, 150])
#     upper_red1 = np.array([10, 255, 255])
#     lower_red2 = np.array([170, 150, 150])
#     upper_red2 = np.array([180, 255, 255])
    
#     # Threshold the HSV image to get only red colors
#     mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
#     mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
#     mask = mask1 + mask2
    
#     # Reduce noise
#     mask = cv2.medianBlur(mask, 5)
    
#     # Find contours
#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
#     for contour in contours:
#         # Approximate a circle using minimum enclosing circle
#         (x, y), radius = cv2.minEnclosingCircle(contour)
#         center = (int(x), int(y))
#         radius = int(radius)
        
#         if radius > 10:  # Minimum radius threshold
#             cv2.circle(frame, center, radius, (0, 255, 0), 2)
#             cv2.circle(frame, center, 2, (255, 0, 0), 3)
#             print(f"Red Circle detected at: {center}")
    
#     cv2.imshow("Red Circle Detection", frame)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

# # Example usage
# detect_red_circle(sys.argv[1])
# # detect_red_circle("rgb_image.png")

# import cv2
# import numpy as np

def detect_red_circle(image_path):
    # Load the image
    frame = cv2.imread(image_path)
    if frame is None:
        print("Error: Unable to load image.")
        return
    
    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define range for red color (adjusted for better detection)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    # Threshold the HSV image to get only red colors
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2
    
    # Apply morphological operations to remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        # Approximate a circle using minimum enclosing circle
        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y))
        radius = int(radius)
        
        if radius > 20:  # Adjusted minimum radius threshold
            cv2.circle(frame, center, radius, (0, 255, 0), 2)
            cv2.circle(frame, center, 2, (255, 0, 0), 3)
            print(f"Red Circle detected at: {center}, Radius: {radius}")
    
    cv2.imshow("Red Circle Detection", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Example usage
# detect_red_circle("rgb_image.jpg")
detect_red_circle(sys.argv[1])