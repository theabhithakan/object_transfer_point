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
from reflex_msgs.msg import PoseCommand 



def main():
    rospy.init_node("OTP_Experiment", disable_signals=True)
    print "init"
    face = head_wobbler.Wobbler()
    base = move_base.MoveBase()
    print "initialized"
    grip_pub = rospy.Publisher("/right_hand/command_position", PoseCommand, queue_size=10)
    drop_pub = rospy.Publisher("/right_hand/command_position", PoseCommand, queue_size=10)
    # base.move_left()
    # base.move_right()
    # base.move_left()
    # stand_right = promp.ProMP("../scripts/DataICRA19/BaselineB")
    # stand_front = promp.ProMP("../scripts/DataICRA19/BaselineB")
    # stand_back = promp.ProMP("../scripts/DataICRA19/BaselineB")
    # sit_left = promp.ProMP("../scripts/DataICRA19/BaselineB")
    # sit_front = promp.ProMP("../scripts/DataICRA19/BaselineB")
    # sit_back = promp.ProMP("../scripts/DataICRA19/BaselineB")
    
    # # take gripper command
    # # drop gripper command


    # text = raw_input("Start 1st case? (y/n)")
    # if text == 'n':
    #     print "ok bye"
    # else:
    #     face.look_right()
    #     stand_right.test_promp()
    #     # take command
    #     stand_right.reset_right_hand()
    #     # drop command
    #     face.set_neutral()
    #     base.move_right()

    # text = raw_input("Start 2nd case? (y/n)")
    # if text == 'n':
    #     print "ok bye"
    # else:
    #     stand_front.test_promp()
    #     # take command
    #     stand_front.reset_right_hand()
    #     # drop command

    # text = raw_input("Start 3rd case? (y/n)")
    # if text == 'n':
    #     print "ok bye"
    # else:
    #     stand_back.test_promp()
    #     # take command
    #     stand_back.reset_right_hand()
    #     # drop command

    # text = raw_input("Reset Robot? (y/n)")
    # if text == 'n':
    #     print "ok bye"
    # else:
    #     base.move_left()
    
    # text = raw_input("Start 4th case? (y/n)")
    # if text == 'n':
    #     print "ok bye"
    # else:
    #     face.look_left()
    #     sit_left.test_promp()
    #     # take command
    #     sit_left.reset_right_hand()
    #     # drop command
    #     face.set_neutral()
    #     base.move_left()
    
    # text = raw_input("Start 5th case? (y/n)")
    # if text == 'n':
    #     print "ok bye"
    # else:
    #     sit_front.test_promp()
    #     # take command
    #     sit_front.reset_right_hand()
    #     # drop command

    # text = raw_input("Start 6th case? (y/n)")
    # if text == 'n':
    #     print "ok bye"
    # else:
    #     sit_back.test_promp()
    #     # take command
    #     sit_back.reset_right_hand()
    #     # drop command

    # text = raw_input("Reset Robot? (y/n)")
    # if text == 'n':
    #     print "ok bye"
    # else:
    #     base.move_right()
    print "starting"
    grip = PoseCommand()
    grip.f1 = 2.2
    grip.f2 = 2.2
    grip.f3 = 2.2
    grip.preshape = 0.0
    # grip.velocity.f1 = 1.0
    # grip.velocity.f2 = 1.0
    # grip.velocity.f3 = 1.0
    # grip.velocity.preshape = 2.0
    start = time.time()
    stop = 4
    while time.time()-start < stop:
            grip_pub.publish(grip)

    drop = PoseCommand()
    drop.f1 = 0.0
    drop.f2 = 0.0
    drop.f3 = 0.0
    drop.preshape = 0.0
    # drop.velocity.f1 = 1.0
    # drop.velocity.f2 = 1.0
    # drop.velocity.f3 = 1.0
    # drop.velocity.preshape = 2.0
    start = time.time()
    stop = 4
    while time.time()-start < stop:
            drop_pub.publish(drop)

    # drop_pub.publish(drop)
    print "done"

if __name__ == '__main__':
    main()