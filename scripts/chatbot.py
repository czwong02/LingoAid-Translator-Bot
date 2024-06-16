#!/usr/bin/env python

# Purpose:
# This script manages user interaction, handles speech input, commands for taking pictures, 
# and speaks out the translated results. It integrates with the speech recognition and image processing nodes.

# Input
# 1. Topic: "processed_speech_result" (Type: std_msgs/String)
# Content: The translated text result of speech input.
# 2. Topic: "processed_image_result" (Type: std_msgs/String)
# Content: The translated text result of text extracted from an image.
# 3. Topic: "/result" (Type: std_msgs/String)
# Content: The recognized text result from Google Speech Recognition.

# Output
# 1. Topic: "take_photo" (Type: std_msgs/String)
# Content: Command to take a photo (e.g., "take photo").
# 2. Topic: "speech_input" (Type: std_msgs/String)
# Content: The recognized text from speech input to be translated.
    
import rospy
from std_msgs.msg import String
from gtts import gTTS
import os

class Chatbot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('chatbot')
        
        # Start with an introductory message
        self.speak('Hello, I am an English to Malay Translator Chatbot. How can I assist you today?')

        # Publisher to trigger taking a photo
        self.take_photo = rospy.Publisher("take_photo", String, queue_size=10)
        
        # Publisher to send processed text or image for translation
        self.process_text_pub = rospy.Publisher("speech_input", String, queue_size=10)
        
        # Subscriber to receive processed speech result
        self.processed_result_sub = rospy.Subscriber("processed_speech_result", String, self.handle_processed_result)
        
        # Subscriber to receive processed image result
        self.processed_image_result_sub = rospy.Subscriber("processed_image_result", String, self.handle_processed_result)

        # Subscriber to receive Google Speech Recognition result
        self.google_sr = rospy.Subscriber("/result", String, self.talkback)

    # Function to convert text to speech
    def speak(self, data):
        tts = gTTS(data)
        tts.save("speech.mp3")
        os.system("mpg321 speech.mp3")
        os.remove("speech.mp3")

    # Callback function to handle Google Speech Recognition result
    def talkback(self, msg):
        rospy.loginfo(msg.data)
        if 'take a picture' in msg.data:
            # Prompt to take a photo
            self.speak("You want to take a photo? Ok, get ready. One, two, three, say cheese")
            # Trigger to take photo
            self.take_photo.publish('take photo')
        else:
            # Publish the received speech for processing
            self.process_text_pub.publish(msg.data)

    # Callback function to handle processed result
    def handle_processed_result(self, msg):
        rospy.loginfo("Processed result received: %s", msg.data)
        # Speak out the processed result
        self.speak(msg.data)

if __name__ == "__main__":
    try:
        # Initialize Chatbot node
        Chatbot()
        # Keep ROS node running
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Chatbot node terminated.")
