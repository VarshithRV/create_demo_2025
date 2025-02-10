import rospy
import cv2
import os
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def image_callback(msg):
    global image_counter, last_saved_time
    try:
        # Convert ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Ensure the output directory exists
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # Save at 30 FPS
        current_time = time.time()
        if current_time - last_saved_time >= 1.0 / 30:
            image_filename = os.path.join(output_dir, f'image_{image_counter:04d}.jpg')
            cv2.imwrite(image_filename, cv_image)
            rospy.loginfo(f"Saved {image_filename}")
            
            image_counter += 1
            last_saved_time = current_time
    except Exception as e:
        rospy.logerr(f"Failed to process image: {e}")

if __name__ == "__main__":
    rospy.init_node('image_saver', anonymous=True)
    
    bridge = CvBridge()
    image_counter = 0
    output_dir = "sampled_images"
    last_saved_time = time.time()
    
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    
    rospy.loginfo("Image saver node started. Waiting for images...")
    rospy.spin()
