import rospy
import numpy as np
import cv2
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

depth_data_list = []
bridge = CvBridge()

def depth_callback(msg):
    global depth_data_list
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        height, width = depth_image.shape
        start_x = (width - 430) // 2
        start_y = (height - 430) // 2
        depth_image = depth_image[start_y:start_y+430, start_x:start_x+430]
        depth_data_list.append(np.array(depth_image, dtype=np.float32))
    except Exception as e:
        rospy.logerr(f"Error converting depth image: {e}")

def compute_rmse():
    global depth_data_list
    depth_stack = np.stack(depth_data_list, axis=0)
    mean_depth = np.mean(depth_stack, axis=0)
    rmse_depth = np.sqrt(np.mean((depth_stack - mean_depth) ** 2, axis=0))
    return rmse_depth

def main():
    rospy.init_node('depth_error_mapping', anonymous=True)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback)
    
    rospy.loginfo("Collecting depth data for 15 seconds...")
    start_time = time.time()
    while time.time() - start_time < 15:
        rospy.sleep(0.1)
    
    rospy.loginfo("Computing depth RMSE...")
    rmse_depth = compute_rmse()
    
    # Statistics
    min_error, max_error = np.min(rmse_depth), np.max(rmse_depth)
    mean_error, std_error = np.mean(rmse_depth), np.std(rmse_depth)
    rospy.loginfo(f"Depth Error Statistics: Min={min_error:.4f}, Max={max_error:.4f}, Mean={mean_error:.4f}, Std={std_error:.4f}")
    
    # Save depth error image
    depth_error_image = (255 * (rmse_depth - min_error) / (max_error - min_error)).astype(np.uint8)
    cv2.imwrite("depth_error_image.png", depth_error_image)
    rospy.loginfo("Saved depth error image as 'depth_error_image.png'")
    
    # Plot heatmap
    plt.figure(figsize=(8, 6))
    plt.imshow(rmse_depth, cmap='jet', interpolation='nearest')
    plt.colorbar(label='Root Mean Squared Error')
    plt.title("Depth Error Heatmap")
    plt.savefig("depth_error_heatmap.png")
    plt.show()
    rospy.loginfo("Displayed depth error heatmap.")
    
    # Plot continuous frequency distribution (Kernel Density Estimate)
    plt.figure(figsize=(8, 6))
    import seaborn as sns
    sns.kdeplot(rmse_depth.flatten(), fill=True, color='blue', alpha=0.7)
    plt.xlabel("Depth Error Value")
    plt.ylabel("Density")
    plt.title("Continuous Frequency Distribution of Depth Error")
    plt.grid()
    plt.savefig("depth_error_distribution.png")
    plt.show()
    rospy.loginfo("Displayed depth error continuous frequency distribution.")
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
