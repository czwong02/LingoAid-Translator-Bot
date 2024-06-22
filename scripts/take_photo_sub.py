#!/usr/bin/env python

# Script for simulation
# Launch gazebo world prior to run this script

# Purpose:
# This script captures a photo using a camera when commanded and publishes 
# the file path of the saved image to a ROS topic for further processing.

# Input:
# 1. Topic: "/usb_cam/image_raw" (Type: sensor_msgs/Image)
# Content: Raw image data from the camera.
# 2. Topic: "/take_photo" (Type: std_msgs/String)
# Content: Command to take a photo (e.g., "take photo").

# Output:
# Topic: "capture_image_word_input" (Type: std_msgs/String)
# Content: The file path of the saved photo.

# Import neccessary libraries
from __future__ import print_function
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

# Class for taking photos
class TakePhoto:
    def __init__(self):

        # Initialize CvBridge object
        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        #img_topic = "/camera/rgb/image_raw"
        img_topic = "/usb_cam/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Publisher for image location
        self.img_location_pub = rospy.Publisher('capture_image_word_input', String, queue_size=50)

        # Allow up to one second for connection
        rospy.sleep(1)

        rospy.Subscriber('/take_photo', String, self.take_photo)

    # Callback function for receiving images
    def callback(self, data):

        # Convert image to OpenCV format
        try:    
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

    # Function to take and save a photo
    def take_picture(self, img_title):
        if self.image_received:
            # Save an image
            img_title = 'catkin_ws/src/lingoaid_translator_bot/img/' + img_title
            cv2.imwrite(img_title, self.image)
            return True, img_title
        else:
            return False, ''

    # Function to handle taking a photo upon receiving a command
    def take_photo(self, msg):
        if msg.data == "take photo":
            # Take a photo
            timestr = time.strftime("%Y%m%d-%H%M%S-")
            img_title = timestr + "photo.jpg"

            # Attempt to save the photo
            success, img_path = self.take_picture(img_title)

            if success:
                rospy.loginfo("Saved image " + img_path)
                img_url = "catkin_ws/src/lingoaid_translator_bot/img/" + img_title
                # Publish the location of the stored image
                self.img_location_pub.publish(img_url)
            else:
                rospy.loginfo("No images received")
 
if __name__ == '__main__':

    # Initialize
    rospy.init_node('take_photo', anonymous=False)
    take_photo_instance = TakePhoto()
    
    rospy.spin()
    
