#!/usr/bin/env python
import sys
import rospy
import time
import numpy as np
from std_msgs.msg import Float64MultiArray
import baxter_interface
import promp




def main():
    rospy.init_node("OTP_Experiment", disable_signals=True)

    stand_right = promp.ProMP("../scripts/DataICRA19/BaselineB")
    stand_front = promp.ProMP("../scripts/DataICRA19/BaselineB")
    stand_back = promp.ProMP("../scripts/DataICRA19/BaselineB")
    sit_left = promp.ProMP("../scripts/DataICRA19/BaselineB")
    sit_front = promp.ProMP("../scripts/DataICRA19/BaselineB")
    sit_back = promp.ProMP("../scripts/DataICRA19/BaselineB")

    stand_right.test_promp()


if __name__ == '__main__':
    main()