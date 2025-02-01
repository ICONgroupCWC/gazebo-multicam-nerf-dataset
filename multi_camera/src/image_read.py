import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
import threading

# Define camera topics
camera_topics = [
    '/camera_1/image_raw',
    '/camera_2/image_raw',
    '/camera_3/image_raw',
    'jb_0/image_raw'  # Robot's camera
]

# Define folder names
camera_folders = {
    '/camera_1/image_raw': 'camera_1',
    '/camera_2/image_raw': 'camera_2',
    '/camera_3/image_raw': 'camera_3',
    'jb_0/image_raw': 'robot'
}

# Create directories if they don't exist
for folder in camera_folders.values():
    os.makedirs(folder, exist_ok=True)

# CvBridge for ROS to OpenCV conversion
bridge = CvBridge()

# Global variables to store image buffer and odometry
image_buffer = {}
robot_pose = None
last_received_time = time.time()
IMAGE_TIMEOUT = 5.0  # Timeout for missing images (seconds)

def image_callback(msg, topic):
    """Store images in buffer and process them when all are received."""
    global last_received_time
    try:
        # Convert ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if cv_image is None or cv_image.size == 0:
            rospy.logwarn(f"Received empty image from {topic}")
            return
        
        image_buffer[topic] = cv_image
        last_received_time = time.time()  # Update timestamp

        rospy.loginfo(f"Received image from {topic}. Buffer size: {len(image_buffer)}/{len(camera_topics)}")

        # If all four images are received, save them
        if len(image_buffer) == len(camera_topics):
            save_images()
    
    except Exception as e:
        rospy.logerr(f"Error processing image from {topic}: {e}")

def save_images():
    """Save images and odometry if all images are available."""
    global image_buffer, robot_pose

    timestamp = time.time()
    file_name = f"{timestamp:.2f}"

    # Debugging: Print which topics are in the buffer
    rospy.loginfo(f"Buffered images: {list(image_buffer.keys())}")

    # Ensure all images exist before saving
    if len(image_buffer) != len(camera_topics):
        rospy.logwarn(f"Incomplete image set (expected {len(camera_topics)}, got {len(image_buffer)}), discarding this round...")
        image_buffer.clear()
        return

    try:
        # Save images to respective folders
        for cam_topic in camera_topics:
            if cam_topic in image_buffer:
                folder = camera_folders[cam_topic]
                image_path = f"{folder}/{file_name}.jpg"
                if cv2.imwrite(image_path, image_buffer[cam_topic]):
                    rospy.loginfo(f"Saved image: {image_path}")
                else:
                    rospy.logwarn(f"Failed to save image: {image_path}")
            else:
                rospy.logerr(f"Image missing for {cam_topic}")

        # Save odometry data only if available
        if robot_pose:
            text_path = f"{camera_folders['jb_0/image_raw']}/{file_name}.txt"
            with open(text_path, 'w') as f:
                f.write(f"Position: {robot_pose['x']}, {robot_pose['y']}, {robot_pose['z']}\n")
                f.write(f"Orientation: {robot_pose['qx']}, {robot_pose['qy']}, {robot_pose['qz']}, {robot_pose['qw']}\n")
            rospy.loginfo(f"Saved odometry: {text_path}")
        else:
            rospy.logwarn("Skipping odometry save, no data received yet.")

        # Clear buffer after saving
        image_buffer.clear()

    except Exception as e:
        rospy.logerr(f"Error saving images or odometry: {e}")
        image_buffer.clear()

def odom_callback(msg):
    """Update robot's odometry information."""
    global robot_pose
    robot_pose = {
        'x': msg.pose.pose.position.x,
        'y': msg.pose.pose.position.y,
        'z': msg.pose.pose.position.z,
        'qx': msg.pose.pose.orientation.x,
        'qy': msg.pose.pose.orientation.y,
        'qz': msg.pose.pose.orientation.z,
        'qw': msg.pose.pose.orientation.w
    }
    rospy.loginfo(f"Updated robot pose: {robot_pose}")

def check_timeout():
    """Periodically check if images have timed out."""
    global last_received_time, image_buffer
    while not rospy.is_shutdown():
        time.sleep(2)  # Check every 2 seconds
        if time.time() - last_received_time > IMAGE_TIMEOUT:  # 5-second timeout
            rospy.logwarn("Timeout reached! Resetting image buffer...")
            image_buffer.clear()

if __name__ == '__main__':
    rospy.init_node('synchronized_camera_saver', anonymous=True)

    # Subscribe to camera topics
    for topic in camera_topics:
        rospy.Subscriber(topic, Image, image_callback, callback_args=topic)

    # Subscribe to odometry topic
    rospy.Subscriber('/jb_0/odom', Odometry, odom_callback)

    # Start timeout checker in background
    timeout_thread = threading.Thread(target=check_timeout)
    timeout_thread.daemon = True
    timeout_thread.start()

    rospy.spin()  # Keep the node running
