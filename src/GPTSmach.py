#!/usr/bin/env python3
import random
import time
import concurrent.futures
import asyncio
import rospy
import text2emotion as te
from utils import aimodel
from std_msgs.msg import String
import nltk
from nltk.tokenize import sent_tokenize, word_tokenize
from nltk.sentiment import SentimentIntensityAnalyzer
import logging
from utils.tools import load_csv_file, save_csv_file
from qt_robot_interface.srv import *
from qt_gspeech_app.srv import *
from GPTBot import Synchronizer, QTChatBot
from writing_control import Writing_Control
# from qt_riva_asr_app.srv import *
import threading
import smach
import pandas as pd
from datetime import datetime
import sys
from logger import Logger

TEST_WRITING = False  # Flag to just test the writing part, no interaction with the child
TEST_CONVERSATION = False  # Flag to just test the conversation part, skip the writing part
# Available_Letter = ['F', 'X', 'H', 'Q', 'S', 'R']
Available_Letter = ['X', 'F', 'Q', 'R']
TEST_LETTER = "R"  # Used for TEST_WRITING
WRITTING_REPEAT_TIMES = 1   # Repeat times for writing the same letter
CONVERSATION_TIME = 60     # set the conversation time
ADDITIONAL_WRITING_TIME = 180  # set the additional writing time

User_name = 'Alex'

class Greeting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['proceed'],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger'])    

    def execute(self, userdata):
        rospy.loginfo("Executing state Greeting")
        greeting_start_time = time.time()
        userdata.logger.log_state_time("Greeting", greeting_start_time)
        if not TEST_WRITING:
            intro_response = userdata.GPTBot.intro()
            userdata.logger.log_conversation(time.time(), "QTrobot", intro_response)
            bad_hearing_response = userdata.GPTBot.explain_bad_hearing()
            userdata.logger.log_conversation(time.time(), "QTrobot", bad_hearing_response)
            ask_name_response = userdata.GPTBot.ask_name()
            userdata.logger.log_conversation(time.time(), "QTrobot", ask_name_response)

            while not rospy.is_shutdown() and not userdata.GPTBot.finish:            
                print('waiting for the name') 
                try:
                    start_time = time.time()   
                    recognize_result = userdata.GPTBot.recognizeQuestion(language=userdata.GPTBot.defaultlanguage, options='', timeout=0)
                    end_time = time.time()
                    api_time = end_time - start_time
                    if recognize_result:
                        userdata.logger.log_conversation(end_time, "Human", recognize_result.transcript)
                        print("recognize_result: ", recognize_result)
                        print("api_time: ", api_time, "input token num: ", len(recognize_result.transcript))
                        userdata.GPTBot.google_speech_data.append((api_time, len(recognize_result.transcript)))
                        break
                    else:
                        print("recognize_result is None")                        
                    if not recognize_result or not recognize_result.transcript:
                        # userdata.GPTBot.bored()
                        continue
                except:
                    continue
            if not User_name:
                userdata.Get_Name_Result = recognize_result.transcript
            else:
                userdata.Get_Name_Result = User_name
            prompt = "We are using the google speech to text api to recognize the name of the people you are talking to, the result is" + userdata.Get_Name_Result + "If you get name, you should greet the person again, \
            if you cannot get the name from the reuslt, you could just use 'my friend' instead of the name. Please act like you are talking to a person rather than acting based on the command and greet the person again by asking what you can do for the person"
            response =  userdata.GPTBot.aimodel.generate(prompt)
            print("response: ", response)
            userdata.GPTBot.talk(response)
            userdata.logger.log_conversation(time.time(), "QTrobot", response)
        else:
            userdata.GPTBot.talk("Greeting")
            userdata.logger.log_conversation(time.time(), "QTrobot", "Greeting")
        return 'proceed'


class Conversation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goodbye', 'writing_loop'],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag'])    
    def execute(self, userdata):
        rospy.loginfo("Executing state Conversation")
        conversation_start_time = time.time() 
        userdata.logger.log_state_time("Conversation", conversation_start_time)
        writing_start_flag = False
        if not TEST_WRITING:
            while not rospy.is_shutdown() and not userdata.GPTBot.finish:   
            # while not rospy.is_shutdown():     
                print('listenting...') 
                try:
                    recognize_result = userdata.GPTBot.recognizeQuestion(language=userdata.GPTBot.defaultlanguage, options='', timeout=0)
                    if recognize_result:
                        print("recognize_result: ", recognize_result)
                    else:
                        print("recognize_result is None")                        
                    if not recognize_result or not recognize_result.transcript:
                        continue
                except:
                    continue
                current_time = time.time()
                used_time = current_time - conversation_start_time
                print("used_time: ", used_time)
                print('Human:', recognize_result.transcript)
                userdata.logger.log_conversation(current_time, "Human", recognize_result.transcript)
                prompt = recognize_result.transcript
                words = word_tokenize(prompt.lower())
                response = None
                if used_time > CONVERSATION_TIME - 10:
                    prompt = "The child's latest response is: " + prompt + \
                        "Now you should suggest to teach the child how to write letters, you should first response to the child's latest response started with 'Hmm,' or 'Ah,' and then response with 'Now let us start to write letters'"
                    writing_start_flag = True
                else:
                    prompt = "The child's latest response is: " + prompt + \
                    "You should answer to the responese, our activity today is to learn how to write letter, but no need to directly move on to the writing part at this moment"
                response = userdata.GPTBot.aimodel.generate(prompt)

                if not response:
                    response = userdata.GPTBot.error_feedback

                refined_response = userdata.GPTBot.refine_sentence(response)
                userdata.logger.log_conversation(current_time, "QTrobot", refined_response)
                userdata.GPTBot.no_guesture_speak(refined_response)

            # userdata.GPTBot.no_guesture_start()
            if userdata.GPTBot.finish and userdata.GPTBot.start_writing:
                return 'writing_loop'
                # if writing_start_flag:
                #     return 'writing_loop'
            # elif userdata.GPTBot.finish:
            #     return 'goodbye'
        else:
            userdata.GPTBot.talk("Conversation")
            userdata.logger.log_conversation(time.time(), "QTrobot", "Conversation")
            return 'writing_loop'
        
class WritingLoop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['additional_writing_start'],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag'])
    def execute(self, userdata):
        rospy.loginfo("Executing state WritingLoop")
        writing_loop_start_time = time.time()
        userdata.logger.log_state_time("WritingLoop", writing_loop_start_time)
        if not TEST_WRITING:
            prompt = "Now you are ready to teach the child how to write these letters: " + str(Available_Letter) + "You are going to teach all of them You should start with 'We will be focusing ' and introducing what you are going to teach, limit in 40 words"
            response =  userdata.GPTBot.aimodel.generate(prompt)
            print("response: ", response)
            userdata.GPTBot.talk(response)
            userdata.logger.log_conversation(time.time(), "QTrobot", response)
        else:
            userdata.GPTBot.talk("Writing Loop")
            userdata.logger.log_conversation(time.time(), "QTrobot", "Writing Loop")

        if not TEST_CONVERSATION:
            for letter in Available_Letter:
                userdata.GPTBot.talk("Here is the letter " + letter)
                userdata.logger.log_conversation(time.time(), "QTrobot", "Here is the letter " + letter)
                if not TEST_WRITING:
                    for i in range(WRITTING_REPEAT_TIMES):
                        userdata.WritingControl.writing_prepare_arm()
                        userdata.WritingControl.writing_execution(letter=letter)
                else:
                    userdata.WritingControl.writing_prepare_arm()
                    userdata.WritingControl.writing_execution(letter=letter)
        return 'additional_writing_start'
        
    
class AdditionalWritingStart(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['additional_writing', 'goodbye'],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag', 'target_letter', 'teaching_times', 'taught_letters', 'additional_writing_start_time', 'writting_loop_end'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag', 'target_letter', 'teaching_times', 'taught_letters', 'additional_writing_start_time', 'writting_loop_end'])
    def execute(self, userdata):
        rospy.loginfo("Executing state AdditionalWritingStart")
        additional_writing_start_time = time.time()
        userdata.logger.log_state_time("AdditionalWritingStart", additional_writing_start_time)
        # userdata.GPTBot.talk("Writing Start")
        if not TEST_WRITING:
            if not userdata.writting_loop_end:
                prompt = "Now you have taught the child how to write letters in" + str(Available_Letter) + \
                " However, the child need for review and consolidation. You should first conclude what you taught and the reason for teaching again started with 'Now we' then ask which letter the child want to learn again by saying 'Which letter'"
                response =  userdata.GPTBot.aimodel.generate(prompt)
                userdata.writting_loop_end = True
            else:
                prompt = "Now you have taught the child how to write letters in" + str(Available_Letter) + \
                "You should ask the child which letter he/she would like to learn again. Remember to remind the child of available letter in " + str(Available_Letter) +" again. You should start with 'Which letter'"
                response =  userdata.GPTBot.aimodel.generate(prompt)
            userdata.additional_writing_start_time = time.time()
            print("response: ", response)
            userdata.GPTBot.talk(response)
            userdata.logger.log_conversation(time.time(), "QTrobot", response)
            # listen to the child's answer
            userdata.GPTBot.finish = False
            while not rospy.is_shutdown() and not userdata.GPTBot.finish:
                print('waiting for the letter') 
                try:
                    start_time = time.time()   
                    recognize_result = userdata.GPTBot.recognizeQuestion(language=userdata.GPTBot.defaultlanguage, options='', timeout=0)
                    end_time = time.time()
                    api_time = end_time - start_time
                    if recognize_result:
                        print("recognize_result: ", recognize_result)
                        print("api_time: ", api_time, "input token num: ", len(recognize_result.transcript))
                        userdata.GPTBot.google_speech_data.append((api_time, len(recognize_result.transcript)))
                        userdata.logger.log_conversation(end_time, "Human", recognize_result.transcript)
                    else:
                        print("recognize_result is None")                        
                    if not recognize_result or not recognize_result.transcript:
                        # userdata.GPTBot.bored()
                        continue
                except:
                    continue
                print('Human:', recognize_result.transcript)
                prompt = recognize_result.transcript
                # userdata.GPTBot.show_sentiment(userdata.GPTBot.get_sentiment(prompt))
                words = word_tokenize(prompt.lower())
                print("words: ", words)

                closing_words = ["bye","goodbye","stop", 'end']
                # if any of the letter in Available_Letter appears in the words, no matter upper or lower case, we will take it as the target letter
                if any(word in Available_Letter for word in words) or any(word in [letter.lower() for letter in Available_Letter] for word in words):
                    for word in words:
                        if word in Available_Letter:
                            userdata.target_letter = word.upper()
                            break
                        elif word in [letter.lower() for letter in Available_Letter]:
                            userdata.target_letter = word.upper()
                            break
                used_time = time.time() - additional_writing_start_time
                print("used_time: ", used_time)
                if any(word in closing_words for word in words) or used_time > ADDITIONAL_WRITING_TIME:
                    if any(word in closing_words for word in words):
                        prompt = "The child want to stop now, express your slight regret only"
                    else: # running out of time
                        prompt = "Now you should conclude today you taugh letters" + str(Available_Letter) + "Please only make conclusion at this stage stated with 'Sorry, now is time for stop, we have learned' and ended with 'I am sure you have learned a lot'"
                    response =  userdata.GPTBot.aimodel.generate(prompt)
                    print("response: ", response)
                    userdata.GPTBot.talk(response)
                    userdata.logger.log_conversation(time.time(), "QTrobot", response)
                    return 'goodbye'

                response = None
                input_prompt = "The speech to text response is: (" + prompt  + ") Your task in this state is to get the letter which is available in" + str(Available_Letter) + \
                "If you get the letter, you should say 'Great, Let us start to write the letter', or you should ask again, explain your bad hearing and verify the answer by the child"

                #     response = results[0]   
                response =  userdata.GPTBot.aimodel.generate(input_prompt)
                print("api_time: ", api_time, "input token num: ", len(words))
                if not response:
                    response = userdata.GPTBot.error_feedback
                if userdata.target_letter:
                    break
                userdata.GPTBot.talk(userdata.GPTBot.refine_sentence(response))
                userdata.logger.log_conversation(time.time(), "QTrobot", response)


        else:
            userdata.GPTBot.talk("Additional Writing Start")
            userdata.logger.log_conversation(time.time(), "QTrobot", "Additional Writing Start")
        userdata.teaching_times+=1
        userdata.WritingFlag = WRITTING_REPEAT_TIMES
        return 'additional_writing'


class AdditionalWriting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['additional_writing_end', "additional_writing"],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag', 'target_letter', 'teaching_times', 'taught_letters'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag', 'target_letter', 'teaching_times', 'taught_letters'])
    def execute(self, userdata):
        rospy.loginfo("Executing state AdditionalWriting")
        additional_writing_time = time.time()
        userdata.logger.log_state_time("AdditionalWriting", additional_writing_time)
        if TEST_WRITING:
            userdata.target_letter = TEST_LETTER

        userdata.GPTBot.talk("Here is the letter you want to learn")
        userdata.logger.log_conversation(time.time(), "QTrobot", "Here is the letter you want to learn")
        #TODO: write the letter 
        userdata.WritingFlag -= 1
        if not TEST_CONVERSATION:
            userdata.WritingControl.writing_prepare_arm()
            userdata.WritingControl.writing_execution(letter=userdata.target_letter)
        
        if userdata.WritingFlag == 0:
            userdata.GPTBot.talk("I finished writing")
            userdata.logger.log_conversation(time.time(), "QTrobot", "I finished writing")
            userdata.taught_letters.append(userdata.target_letter)
            return 'additional_writing_end'
        else:
            return 'additional_writing'
        
class AdditionalWritingEnd(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goodbye', 'additional_writing_start'],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag', 'target_letter', 'teaching_times', 'taught_letters', 'additional_writing_start_time'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag', 'target_letter', 'teaching_times', 'taught_letters', 'additional_writing_start_time'])
        
    def execute(self, userdata):
        rospy.loginfo("Executing state AdditionalWritingEnd")
        additional_writing_end_time = time.time()
        userdata.logger.log_state_time("AdditionalWritingEnd", additional_writing_end_time)
        userdata.target_letter = None

        if not TEST_WRITING:
            used_time = additional_writing_end_time - userdata.additional_writing_start_time
            print("used_time for adi: ", used_time)
            if used_time > ADDITIONAL_WRITING_TIME:
                prompt = "Now you should conclude today you taugh letters" + str(Available_Letter) + "Please only make conclusion at this stage stated with 'Now is time for stop, we have learned' and ended with 'I am sure you have learned a lot'"
                response =  userdata.GPTBot.aimodel.generate(prompt)
                print("response: ", response)
                userdata.GPTBot.talk(response)
                userdata.logger.log_conversation(time.time(), "QTrobot", response)
                return 'goodbye'
            else:
                prompt = "Now you have written the letter. Keep the response short and simple, you should ask the child if they want to learn another letter again or stop for today, please start with 'Cheers, ' or 'Great, '."
                response =  userdata.GPTBot.aimodel.generate(prompt)
                print("response: ", response)
                userdata.GPTBot.talk(response)
                userdata.logger.log_conversation(time.time(), "QTrobot", response)
                userdata.GPTBot.finish = False
                # listen to the child's answer
                while not rospy.is_shutdown():
                    while not rospy.is_shutdown() and not userdata.GPTBot.finish:
                        print('waiting for the answer') 
                        try:
                            start_time = time.time()   
                            recognize_result = userdata.GPTBot.recognizeQuestion(language=userdata.GPTBot.defaultlanguage, options='', timeout=0)
                            end_time = time.time()
                            api_time = end_time - start_time
                            if recognize_result:
                                print("recognize_result: ", recognize_result)
                                print("api_time: ", api_time, "input token num: ", len(recognize_result.transcript))
                                userdata.GPTBot.google_speech_data.append((api_time, len(recognize_result.transcript)))
                                userdata.logger.log_conversation(end_time, "Human", recognize_result.transcript)
                                break
                            else:
                                print("recognize_result is None")                        
                            if not recognize_result or not recognize_result.transcript:
                                # userdata.GPTBot.bored()
                                continue
                        except:
                            continue
                    used_time = time.time() - userdata.additional_writing_start_time
                    print("used_time: ", used_time)

                    if used_time > ADDITIONAL_WRITING_TIME:
                        prompt = "Now you should conclude today you taugh letters" + str(Available_Letter) + " NO QUESTION any more. Please only make conclusion at this stage stated with 'Now is time for stop, we have learned' and ended with 'I am sure you have learned a lot'."
                        response =  userdata.GPTBot.aimodel.generate(prompt)
                        print("response: ", response)
                        userdata.GPTBot.talk(response)
                        userdata.logger.log_conversation(time.time(), "QTrobot", response)
                        return 'goodbye'

                    prompt = recognize_result.transcript
                    words = word_tokenize(prompt.lower())

                    closing_words = ["bye","goodbye","stop"]
                    continue_words = ["continue", "another", "more", "next", "other", 'No', 'no']
                    if any(word in closing_words for word in words):
                        return 'goodbye'
                    elif any(word in continue_words for word in words) or any(word in Available_Letter for word in words) or any(word in [letter.lower() for letter in Available_Letter] for word in words):
                        prompt = "Now the chidren want to learn another letter, you should say start with 'OK, Let us'"
                        response =  userdata.GPTBot.aimodel.generate(prompt)
                        print("response: ", response)
                        userdata.GPTBot.talk(response)
                        userdata.logger.log_conversation(time.time(), "QTrobot", response)
                        return 'additional_writing_start'
                    else:
                        prompt = "The speech to text response is: (" + prompt  + "), it is neither a closing word nor a continue word, you should ask again whether the child like to learn another letter, explain your bad hearing and verify the answer by the child"
                        response =  userdata.GPTBot.aimodel.generate(prompt)
                        print("response: ", response)
                        userdata.GPTBot.talk(response)
                        userdata.logger.log_conversation(time.time(), "QTrobot", response)

        else:
            if userdata.WritingFlag == 0:
                userdata.GPTBot.talk("Additional Writing End")
                userdata.logger.log_conversation(time.time(), "QTrobot", "Additional Writing End")
                return 'goodbye'
            else:
                return 'additional_writing_start'
    


class Goodbye(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end'],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag','taught_letters'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag','taught_letters'])
    def execute(self, userdata):
        rospy.loginfo("Executing state Goodbye")
        goodbye_start_time = time.time()
        userdata.logger.log_state_time("Goodbye", goodbye_start_time)
        userdata.WritingControl.writing_end_arm()
        if not TEST_WRITING:
            prompt = "Now we are at the goodbye stage. We are using the google speech to text api to recognize the name of the people you are talking to, the result is" + userdata.Get_Name_Result + "If you get name, you should express your thanks\
            to the person you are talking to, if you cannot get the name, you can just say 'my friend' instead of the name. Please act like you are talking to a person rather than acting based on the command and express your thanks to the person you are talking to. and conclude today you taugh letters" + \
            str(Available_Letter) + "The conversation will end after you made conclusion and express your thanks"
            response =  userdata.GPTBot.aimodel.generate(prompt)
            print("response: ", response)
            userdata.GPTBot.talk(response)
            userdata.logger.log_conversation(time.time(), "QTrobot", response)
        else:
            userdata.GPTBot.talk("Goodbye")
            userdata.logger.log_conversation(time.time(), "QTrobot", "Goodbye")
        userdata.logger.save_state_time_to_csv()
        userdata.logger.save_conversation_to_csv()
        return 'end'


class GPTSmachManager():
    def __init__(self):
        # Initialize ROS node
        print("GPTSMACH INIT")
        rospy.init_node('qt_gpt_smach', anonymous=True)
        self.sm = smach.StateMachine(outcomes=['end'])
        self.sm.userdata.gpt_bot = QTChatBot()  # Initialize the GPTBot instance
        self.sm.userdata.get_name_result = ''
        self.sm.userdata.writing_control = Writing_Control()
        self.sm.userdata.writing_flag = 2
        self.sm.userdata.target_letter = None
        self.sm.userdata.writting_loop_end = False
        self.sm.userdata.teaching_times = 0
        self.sm.userdata.taught_letters = []
        self.sm.userdata.logger = Logger()
        self.sm.userdata.additional_writing_start_time = 0

        now = datetime.now()
        dt_string = now.strftime("%d-%m-%Y_%H-%M-%S")
        self.sm.userdata.writing_control.publish_signal("clear_trajectory")

        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('GREETING', Greeting(),
                                transitions={'proceed':'CONVERSATION'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WritingControl':'writing_control', 'WritingFlag':'writing_flag'})
            smach.StateMachine.add('CONVERSATION', Conversation(),
                                transitions={'goodbye':'GOODBYE', 'writing_loop':'WRITING_LOOP'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WritingControl':'writing_control', 'WritingFlag':'writing_flag'})
            smach.StateMachine.add('GOODBYE', Goodbye(),
                                transitions={'end':'end'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WritingControl':'writing_control', 'WritingFlag':'writing_flag'})
            smach.StateMachine.add('WRITING_LOOP', WritingLoop(),
                                transitions={'additional_writing_start':'ADDITIONAL_WRITING_START'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WritingControl':'writing_control', 'WritingFlag':'writing_flag'})
            smach.StateMachine.add('ADDITIONAL_WRITING_START', AdditionalWritingStart(),
                                transitions={'additional_writing':'ADDITIONAL_WRITING', 'goodbye':'GOODBYE'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WritingControl':'writing_control', 'WritingFlag':'writing_flag'})
            smach.StateMachine.add('ADDITIONAL_WRITING', AdditionalWriting(),    
                                transitions={'additional_writing_end':'ADDITIONAL_WRITING_END', 'additional_writing':'ADDITIONAL_WRITING'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WritingControl':'writing_control', 'WritingFlag':'writing_flag'})
            smach.StateMachine.add('ADDITIONAL_WRITING_END', AdditionalWritingEnd(),
                                transitions={'goodbye':'GOODBYE', 'additional_writing_start':'ADDITIONAL_WRITING_START'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WritingControl':'writing_control', 'WritingFlag':'writing_flag'})
            

        self.sm.set_initial_state(['GREETING'])

        # Execute SMACH plan
        outcome = self.sm.execute()
        rospy.loginfo("OUTCOME: " + outcome)
        # rospy.spin()



if __name__ == "__main__":
    print("GPTSMACH TEST")
    try:
        myGPTSmachManager = GPTSmachManager()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Ctrl+C pressed. Shutting down...")


# interview questions
        
