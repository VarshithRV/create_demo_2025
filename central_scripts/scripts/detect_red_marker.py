import cv2
import numpy as np

def detect_red_circle(image_path):
    # Load the image
    frame = cv2.imread(image_path)
    if frame is None:
        print("Error: Unable to load image.")
        return
    
    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define range for red color (filled red color detection)
    lower_red1 = np.array([0, 150, 150])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 150, 150])
    upper_red2 = np.array([180, 255, 255])
    
    # Threshold the HSV image to get only red colors
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2
    
    # Reduce noise
    mask = cv2.medianBlur(mask, 5)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        # Approximate a circle using minimum enclosing circle
        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y))
        radius = int(radius)
        
        if radius > 10:  # Minimum radius threshold
            cv2.circle(frame, center, radius, (0, 255, 0), 2)
            cv2.circle(frame, center, 2, (255, 0, 0), 3)
            print(f"Red Circle detected at: {center}")
    
    cv2.imshow("Red Circle Detection", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Example usage
detect_red_circle("redcircle.png")
