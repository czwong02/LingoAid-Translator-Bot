
  
  

# LingoAid Translator Bot

  

## Background

Many elderly people or senior citizens in Malaysia may not be fluent in English / Malay, which is often required for accessing medical services, government information, or dealing with customer service representatives. Understanding the official documents, instructions, and services provided by government agencies can be a challenging task for them if these are not available in their native language. Plus, simple daily activities like shopping, banking, or using public transportation can become stressful if language becomes a barrier.

  

Our robot aims to break the language barrier by providing the speech and document translation features to these elderly individuals. For example, the robot will detect the human voice from the conversations between elderly individuals and healthcare providers, ensuring they understand medical advice and prescriptions. The robot also can scan and translate written documents, for example, some elderly individuals might not be able to see the expiry date on the product they buy clearly, they can choose to snap the product using the camera, and the robot can capture the text and translate into the user's native language.

  

## Things to take note

There is missing API KEY that need to add before running the application. The python files involve:

- capture_image_word_translator.py

- speech_translator.py

  

Example:

OPENAI_API_KEY = *< REPLACE WITH OPENAI_API_KEY>*

  

Therefore, if you want to run this application in Jupiter robot, you can contact group leader - +60 13-958 0338 (Rain Poo) to obtain the API KEY.

  

Besides, usb_cam-test.launch file is used in this application but it didn't include in this github codebase. Therefore, please reuse the one that exist in the Jupiter robot.

  

> **WARNING!!!**
Kindly setup the ROS workspace before you clone the source code to your robot. If there is no existing ROS workspace can be used, please create a new workspace.

Steps to setup new ROS workspace
1. Create a folder *<YOUR_ROS_WORKSPACE>* in home directory.
    

>     mkdir <YOUR_ROS_WORKSPACE> #Replace it with a proper name

2. Create a folder src in the *<YOUR_ROS_WORKSPACE>* folder.

>     cd <YOUR_ROS_WORKSPACE>
>     mkdir src

3. Run catkin_make in *<YOUR_ROS_WORKSPACE>* folder.

> catkin_make

4. Setting the terminals

>     cd devel/
>     source setup.bash
>     vim ~/.bashrc

For vim ~/.bashrc, please ensure these two lines exist :

    source /opt/ros/*<YOUR_ROS_VERSION>*/setup.bash
    source ~/*<YOUR_ROS_WORKSPACE>*/devel/setup.bash

