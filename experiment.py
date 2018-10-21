#!/usr/bin/env python
import sys
import rospy
import time
import numpy as np
from std_msgs.msg import Float64MultiArray
import baxter_interface
import promp
import move_base
import head_wobbler



def main():
    rospy.init_node("OTP_Experiment", disable_signals=True)

    face = head_wobbler.Wobbler()
    base = move_base.MoveBase()
    # base.move_left()
    # base.move_right()
    # base.move_left()
    stand_right = promp.ProMP("../scripts/DataICRA19/BaselineB")
    stand_front = promp.ProMP("../scripts/DataICRA19/BaselineB")
    stand_back = promp.ProMP("../scripts/DataICRA19/BaselineB")
    sit_left = promp.ProMP("../scripts/DataICRA19/BaselineB")
    sit_front = promp.ProMP("../scripts/DataICRA19/BaselineB")
    sit_back = promp.ProMP("../scripts/DataICRA19/BaselineB")
    
    # take gripper command
    # drop gripper command

    text = raw_input("Start 1st case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        face.look_right()
        stand_right.test_promp()
        # take command
        stand_right.reset_right_hand()
        # drop command
        face.set_neutral()
        base.move_right()

    text = raw_input("Start 2nd case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        stand_front.test_promp()
        # take command
        stand_front.reset_right_hand()
        # drop command

    text = raw_input("Start 3rd case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        stand_back.test_promp()
        # take command
        stand_back.reset_right_hand()
        # drop command

    text = raw_input("Reset Robot? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        base.move_left()
    
    text = raw_input("Start 4th case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        face.look_left()
        sit_left.test_promp()
        # take command
        sit_left.reset_right_hand()
        # drop command
        face.set_neutral()
        base.move_left()
    
    text = raw_input("Start 5th case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        sit_front.test_promp()
        # take command
        sit_front.reset_right_hand()
        # drop command

    text = raw_input("Start 6th case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        sit_back.test_promp()
        # take command
        sit_back.reset_right_hand()
        # drop command

    text = raw_input("Reset Robot? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        base.move_right()

if __name__ == '__main__':
    main()