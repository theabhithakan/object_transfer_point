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


    face.look_right()
    stand_right.test_promp()
    stand_right.reset_right_hand()
    face.set_neutral()

    base.move_right()
    stand_front.test_promp()
    stand_front.reset_right_hand()

    stand_back.test_promp()
    stand_back.reset_right_hand()

    base.move_left()
    face.look_left()
    sit_left.test_promp()
    sit_left.reset_right_hand()
    face.set_neutral()

    base.move_left()
    sit_left.test_promp()
    sit_left.reset_right_hand()

    sit_left.test_promp()
    sit_left.reset_right_hand()    
    base.move_right()

if __name__ == '__main__':
    main()