# QTrobot GPT-Smach

###### Implementing a simple speech-based chatbot with OpenAI and QTrobot

## Relevant ROS package:

All rospackages are available in [luxai-qtrobot](https://github.com/luxai-qtrobot), you can first clone both *software* and *tutorials* to your local device.

You should link or copy the following package to your src under catkin_ws and run the catkin_make
+ *headers*
+ *motors_moveit*
+ *qt_gspeech_app*
+ *qtrobot_ikfast_left_arm_plugin*
+ *qtrobot_ikfast_right_arm_plugin*
+ *audio_common* (in case something goes wrong with audio, can be found [here](https://github.com/ros-drivers/audio_common))

## Key Requirements 

- *qt_gspeech_app* running for speech recognition, check setup insturctions [here](https://github.com/luxai-qtrobot/software/tree/master/apps/qt_gspeech_app)
- *motors_moveit* running for arm control, see more instruction [here](https://docs.luxai.com/docs/v1/tutorials/python/python_ros_moveit)
- *openai credentials* for accessing OpenAI GPT models 

## Other useful documents
+ [Controlling QTrobot arms using MoveIt](https://docs.luxai.com/docs/v1/tutorials/python/python_ros_moveit)
+ [A complete guide to build a Conversational Social Robot with QTrobot and ChatGPT ](https://luxai.com/blog/complete-guide-to-build-conversational-social-robot-qtrobot-chatgpt/)
+ [QTrobot Motion and Actuators](https://docs.luxai.com/docs/v1/modules/motors)
+ [QTrobot Audio processing and Microphone](https://docs.luxai.com/docs/modules/microphone)
+ [SMACH Tutorials](https://wiki.ros.org/smach/Tutorials)

Note that you may also see *qt_gspeech_interface* for speech recognition. It could be another solution but is not tested. 

## Installation 
Install the required python packages:

```
sudo pip3 install -r requirements.txt
```

## Configuration
Add OpenAI key to gpt_bot.yaml file:

```bash
"OPENAI_KEY":""
```

To use OpenAI *gpt-3.5-turbo* model set  `engine: "chatgpt"` or if you want to use *text-davinci-003* set it to `engine: "davinci3"`. Remember do not push you OPENAI_KEY to the repo.

### Character configuration

There are 5 pre-configured characters for 'gpt-3.5-turbo' you can use (qtrobot, fisherman, astronaut, therapist and gollum). Simply change in the 'gpt_bot.yaml' character parameter on line 12 and have fun talking with them.

To use custom character prompt. Change the parameter 'use_prompt' to true and write your 'prompt' instead of the template that is in the 'gpt_bot.yaml'. QTrobot will then take your prompt to get reposnes from GPT.

### Build 
make a link (or copy) to `gpt_smach` in catkin workspace source directory and build it.

```
$ cd ~/catkin_ws/src
$ ln -s ~/robot/code/tutorials/examples/gpt_smach ./
$ cd ~/catkin_ws/
$ catkin_make
```

## Launching QTrobot GPT Smach rosnode

Make sure you are connecting to QTrobot's Wi-Fi hot spot and have internet access

Launch the motors_moveit for control

```
$ roslaunch motors_moveit moveit_qtrobot.launch
```

Launch the visualization process

```
$ roslaunch gpt_smach visualize.launch
```

Launch the gpt_smach main process

```
$ roslaunch gpt_smach gpt_smach.launch
```

