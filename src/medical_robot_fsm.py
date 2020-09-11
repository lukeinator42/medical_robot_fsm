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

def keyword_spotted_transition(args):
    print("State 1: listening for keyword")

    found = False

    while not found:
        res = rospy.wait_for_message("/ros_keyword_spotting/recognition", Keyword)
        print("label: ", res.label)

        if res.label == "robot":
            found = True

    new_state = "keyword_detected"

    return (new_state, None)


def delay_transition(args):
    print("State 2: keyword detected, 4 second delay")

    sleep(4)

    new_state = "user_processing"

    return (new_state, None)



while not rospy.is_shutdown():
    m = StateMachine(rospy)
    m.add_state("start", keyword_spotted_transition)
    m.add_state("keyword_detected", delay_transition)
    m.add_state("user_processing", None, end_state=1)
    # m.add_state("not_state", not_state_transitions)
    #
    # m.add_state("pos_state", None, end_state=1)
    # m.add_state("error_state", None, end_state=1)
    m.set_start("start")
    print("starting state machine")
    m.run(None)
    print("run complete!")


