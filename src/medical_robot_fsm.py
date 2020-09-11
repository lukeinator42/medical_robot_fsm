#!/usr/bin/env python
import time
from threading import Thread
import rospy
from std_msgs.msg import String
from ros_keyword_spotting.msg import Keyword
import numpy as np
from time import sleep
import subprocess

from statemachine import StateMachine

rospy.init_node('medical_robot_fsm_node', anonymous=True)

tts_topic_name = rospy.get_param('~topic_name')
rospy.loginfo("%s is %s", rospy.resolve_name('medical_robot_fsm_node'), tts_topic_name)

loop_rate = rospy.get_param('loop_rate')
rospy.loginfo("%s is %s", rospy.resolve_name('loop_rate'), loop_rate)

rate = rospy.Rate(loop_rate)  # 10hz

tts_pub = rospy.Publisher('/ros_tts/tts', String, queue_size=10)

def keyword_spotted_transition(args):
    print("State 1: listening for keyword")

    found = False

    while not found:
        res = rospy.wait_for_message("/keyword_spotting/recognition", Keyword)
        print("label: ", res.label)

        if res.label == "robot":
            found = True

    new_state = "keyword_detected"

    return (new_state, None)


def delay_transition(args):
    print("State 2: keyword detected, 4 second delay")

    sleep(4)

    tts_pub.publish("I am processing your request. Please wait.")
    rate.sleep()

    new_state = "user_processing"

    return (new_state, None)


def tell_user_processing_transition(args):
    print("State 3: telling user")

    new_state = "parse_result"

    return(new_state, None)


def parse_result_transition(args):
    print("State 4: parsing result")

    res = rospy.wait_for_message("/speech_recognition/recognized_text", String)
    res_str = str(res)

    if "deliver this package to room" not in res_str:
        return ("error_state", None)

    room = res_str.split("deliver this package to ")[1]

    new_state = "confirm_result"

    return(new_state, room)

def confirm_result_transition(room):
    print("State 5: confirming result")

    tts_pub.publish(f"I am supposed to deliver this package to {room}. Is this correct?")

    confirmation = False
    confirmed = False

    while not confirmed:
        res = rospy.wait_for_message("/ros_keyword_spotting/recognition", Keyword)
        print("label: ", res.label)

        if res.label == "yes":
            confirmation = True
            confirmed = True

            tts_pub.publish("Very well. Off I go!")

        elif res.label == "no":
            confirmed = True

    if confirmation:
        new_state = "send_location"

    else:
        new_state = "error_state"

    return (new_state, room)

def send_location_transition(room):
    print("State 6: sending location")

    print(f"Sending {room}")

def error_state_transition(args):
    print("ERRRORRRR!!!!")
    tts_pub.publish(f"Sorry, but I did not understand. Please try again.")

    return("error_final", None)

while not rospy.is_shutdown():
    m = StateMachine(rospy)
    m.add_state("start", keyword_spotted_transition)
    m.add_state("keyword_detected", delay_transition)
    m.add_state("user_processing", tell_user_processing_transition)
    m.add_state("parse_result", parse_result_transition)
    m.add_state("confirm_result", confirm_result_transition)
    m.add_state("send_location", send_location_transition, end_state=1)
    m.add_state("error_state", error_state_transition)
    m.add_state("error_final", None, end_state=1)
    m.set_start("start")
    print("starting state machine")
    m.run(None)
    print("run complete!")


