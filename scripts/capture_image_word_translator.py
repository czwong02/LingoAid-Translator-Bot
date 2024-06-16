#!/usr/bin/env python

# Purpose: 
# This script captures an image, extracts English words from the image, 
# and translates them into Malay using the OpenAI API.

# Input:
# Topic: "capture_image_word_input" (Type: std_msgs/String)
# Content: A string representing the file path or URL of the captured image to be processed.
    
# Output:
# Topic: "processed_image_result" (Type: std_msgs/String)
# Content: The translated text result in Malay, extracted from the image.

import rospy
from std_msgs.msg import String
import requests
import json
import cv2
import base64

class CaptureImageWordTranslator:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('capture_image_word_translator')
        
        # Subscribe to the capture_image_word_input topic
        self.process_image_sub = rospy.Subscriber("capture_image_word_input", String, self.process_image_callback)
        
        # Publisher for processed image result
        self.response_pub = rospy.Publisher("processed_image_result", String, queue_size=10)

    # Callback function for processing image
    def process_image_callback(self, msg):
        # Log the received processed text
        rospy.loginfo("Received processed text: %s", msg.data)
        
        # Call simple_chat function to get response
        response = simple_chat(msg.data)
        if response:
            # Publish the response
            self.response_pub.publish(response)

# Function to encode image to base64
def encode_image(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode("utf-8")

# Function to interact with OpenAI GPT-4 for simple chat using image input
def simple_chat(image_url):
    # Encode image to base64
    base64_image = encode_image(image_url)
        
    # OpenAI API key
    OPENAI_API_KEY = <REPLACE WITH OPENAI_API_KEY>

    # API endpoint for chat completions
    url = "https://api.openai.com/v1/chat/completions"
    
    # Headers for API request
    headers = {
        "Authorization": "Bearer " + OPENAI_API_KEY,
        "Content-Type": "application/json"
    }
    
    # Data to be sent to the API
    data = {
        "model": "gpt-4o",
        "messages": [
            {
                "role": "system",
                "content": "Please capture the word inside the picture. Then translate the word from eng to bm. If no word is captured, please politely response to ask user try again in English."
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": "Please capture the word inside the picture. Then translate the word from eng to bm"
                    },
                    {
                        "type": "image_url",
                        "image_url": { 'url': 'data:image/png;base64,{}'.format(base64_image) }
                    }
                ]
            }
        ], 
        "max_tokens": 300
    }

    # Make POST request to OpenAI API
    response = requests.post(url, headers=headers, data=json.dumps(data))

    # Check if request was successful
    if response.status_code == 200:
        # Parse response JSON
        response_data = response.json()
        # Extract the answer from response
        answer = response_data["choices"][0]["message"]["content"]
        rospy.loginfo("Response: %s", answer)
        return answer
    else:
        # Log error if failed to get response from API
        response_data = response.json()
        rospy.logerr("Failed to get response from chat model: %s", response_data)
        return None

if __name__ == "__main__":
    try:
        # Initialize CaptureImageWordTranslator node
        CaptureImageWordTranslator()
        # Keep ROS node running
        rospy.spin()
    except rospy.ROSInterruptException:
        # Log if CaptureImageWordTranslator node terminated
        rospy.loginfo("CaptureImageWordTranslator node terminated.")
