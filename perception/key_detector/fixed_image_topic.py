#!/usr/bin/env python
# Not for conversion, only for testing

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def publish_image():
    # Initialize the ROS node
    rospy.init_node('image_publisher', anonymous=True)
    
    # Create a publisher that sends images to the /long_range_cam/image topic
    image_pub = rospy.Publisher('/long_range_cam/image', Image, queue_size=10)

    # Load a fixed image (make sure the path is correct)
    fixed_image = cv2.imread('/keyboard.png')

    # Convert the image to the correct ROS format (sensor_msgs/Image)
    bridge = CvBridge()
    ros_image = bridge.cv2_to_imgmsg(fixed_image, encoding="bgr8")

    # Set a rate for publishing
    rate = rospy.Rate(1)  # 1 Hz, change as needed
    
    while not rospy.is_shutdown():
        # Publish the image
        image_pub.publish(ros_image)
        rospy.loginfo("Publishing image")
        
        # Sleep for the rate duration
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass