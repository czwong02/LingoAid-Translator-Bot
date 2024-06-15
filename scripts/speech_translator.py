#!/usr/bin/env python

# Purpose of speech_translator.py: to convert english input into malay input by sending request to openai

import rospy
from std_msgs.msg import String
import requests
import json

class SpeechTranslator:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('speech_translator')
        
        # Subscribe to the speech input topic
        self.process_text_sub = rospy.Subscriber("speech_input", String, self.process_text_callback)
        
        # Publish the processed speech result
        self.response_pub = rospy.Publisher("processed_speech_result", String, queue_size=10)

    # Callback function for processing text
    def process_text_callback(self, msg):
        # Log the received processed text
        rospy.loginfo("Received processed text: %s", msg.data)
        
        # Call simple_chat function to get response
        response = simple_chat(msg.data)
        if response:
            # Publish the response
            self.response_pub.publish(response)
        

# Function to interact with OpenAI GPT-3.5-turbo for simple chat
def simple_chat(text):
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
        "model": "gpt-3.5-turbo",
        "messages": [
            {
                "role": "system",
                "content": "You are an English to Malay translator assistant, answer the user input with 'The translated word is:' + response. If you didn't understand the user input, response politely to ask user to ask again in English."
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": text
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
        
        # Log the unexpected response format
        rospy.loginfo("Response: %s", answer)
        
        # Return the answer
        return answer
    else:
        # Log error if failed to get response from API
        rospy.logerr("Failed to get response from chat model: %s", response.text)
        return None

# Main function
if __name__ == "__main__":
    try:
        # Initialize SpeechTranslator node
        SpeechTranslator()
        
        # Keep ROS node running
        rospy.spin()
    except rospy.ROSInterruptException:
        # Log if SpeechTranslator node terminated
        rospy.loginfo("SpeechTranslator node terminated.")
