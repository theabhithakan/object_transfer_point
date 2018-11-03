#!/usr/bin/env python
import sys
import rospy
import time
import math
import numpy as np
from std_msgs.msg import Float64MultiArray
import baxter_interface
from handover.msg import skeleton
import promp
import move_base
import head_wobbler
from reflex_msgs.msg import Command 



class HumanSaver:
    """docstring for HumanSaver"""
    def __init__(self, height, arm_length, condition):

        self.saver = {}
        self.saver["height"] = height
        self.saver["arm_length"] = arm_length
        self.saver["condition"] = condition

    def callback(self,data):
        self.P_rs = np.array([data.joints[0].x, data.joints[0].y, data.joints[0].z])
        self.P_ls = np.array([data.joints[1].x, data.joints[1].y, data.joints[1].z])
        self.P_rw = np.array([data.joints[2].x, data.joints[2].y, data.joints[2].z])
        self.P_lw = np.array([data.joints[3].x, data.joints[3].y, data.joints[3].z])
        self.P_h = np.array([data.joints[4].x, data.joints[4].y, data.joints[4].z])


    def pointingDirection(self):
        angle = np.arctan2(np.linalg.norm(self.P_rs[1] - self.P_rw[1]), np.linalg.norm(self.P_rs[0:2:3] - self.P_rw[0:2:3]))
        
        return np.rad2deg(angle)

def main():
    rospy.init_node("OTP_Experiment", disable_signals=True)
    grasp_pub = rospy.Publisher("/right_hand/command", Command, queue_size=10)
    hs = HumanSaver(1.4,2,"right")

    face = head_wobbler.Wobbler()
    base = move_base.MoveBase()

    obs_sub = rospy.Subscriber("skeleton_data", skeleton, hs.callback, queue_size = 1)
    time.sleep(2.0)
    hs.pointingDirection()
    
    stand_right = promp.ProMP("src/handover/object_transfer_point/data/stand_right")
    # stand_right = promp.ProMP("../scripts/DataICRA19/BaselineB")
    # stand_front = promp.ProMP("../scripts/DataICRA19/BaselineB")
    # stand_back = promp.ProMP("../scripts/DataICRA19/BaselineB")
    # sit_left = promp.ProMP("../scripts/DataICRA19/BaselineB")
    # sit_front = promp.ProMP("../scripts/DataICRA19/BaselineB")
    # sit_back = promp.ProMP("../scripts/DataICRA19/BaselineB")

    stand_right.reset_right_hand()
    
    take = Command()
    take.pose.f1 = 2.0
    take.pose.f2 = 2.0
    take.pose.f3 = 2.0
    take.pose.preshape = 0.0
    take.velocity.f1 = 3.0
    take.velocity.f2 = 3.0
    take.velocity.f3 = 3.0
    take.velocity.preshape = 2.0
    # time.sleep(2.0)
    # grasp_pub.publish(take)
    # time.sleep(2.0)

    drop = Command()
    drop.pose.f1 = 0.0
    drop.pose.f2 = 0.0
    drop.pose.f3 = 0.0
    drop.pose.preshape = 0.0
    drop.velocity.f1 = 3.0
    drop.velocity.f2 = 3.0
    drop.velocity.f3 = 3.0
    drop.velocity.preshape = 2.0
    # time.sleep(2.0)
    # grasp_pub.publish(drop)
    # time.sleep(2.0)


    # face.look_right()
    # stand_right.runPromp()
    # time.sleep(2.0)
    # grasp_pub.publish(take)
    # time.sleep(4.0)
    # stand_right.safe_dist()
    # stand_right.to_the_basket()
    # time.sleep(2.0)
    grasp_pub.publish(drop)
    # time.sleep(2.0)
    face.set_neutral()
    stand_right.reset_right_hand()

    text = raw_input("Start 1st case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        face.look_right()
        stand_right.test_promp()
        time.sleep(2.0)
        grasp_pub.publish(take)
        time.sleep(4.0)
        stand_right.safe_dist()
        stand_right.to_the_basket()
        time.sleep(2.0)
        grasp_pub.publish(drop)
        # time.sleep(2.0)
        face.set_neutral()
        stand_right.reset_right_hand()

    text = raw_input("Safe to rotate? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        base.move_right()

    text = raw_input("Start 2nd case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        stand_front.test_promp()
        time.sleep(2.0)
        grasp_pub.publish(take)
        time.sleep(2.0)
        stand_front.reset_right_hand()
        time.sleep(2.0)
        grasp_pub.publish(drop)
        time.sleep(2.0)

    text = raw_input("Start 3rd case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        stand_back.test_promp()
        time.sleep(2.0)
        grasp_pub.publish(take)
        time.sleep(2.0)
        stand_back.reset_right_hand()
        time.sleep(2.0)
        grasp_pub.publish(drop)
        time.sleep(2.0)

    text = raw_input("Safe to rotate? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        base.move_left()
    
    text = raw_input("Safe to rotate? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        base.move_left()

    text = raw_input("Safe to rotate? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        base.move_left()

    text = raw_input("Start 4th case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        face.look_right()
        sit_left.test_promp()
        time.sleep(2.0)
        grasp_pub.publish(take)
        time.sleep(2.0)
        sit_left.reset_right_hand()
        time.sleep(2.0)
        grasp_pub.publish(drop)
        time.sleep(2.0)
        face.set_neutral()
        base.move_right()
    
    text = raw_input("Start 5th case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        sit_front.test_promp()
        time.sleep(2.0)
        grasp_pub.publish(take)
        time.sleep(2.0)
        sit_front.reset_right_hand()
        time.sleep(2.0)
        grasp_pub.publish(drop)
        time.sleep(2.0)

    text = raw_input("Start 6th case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        sit_back.test_promp()
        time.sleep(2.0)
        grasp_pub.publish(take)
        time.sleep(2.0)
        sit_back.reset_right_hand()
        time.sleep(2.0)
        grasp_pub.publish(drop)
        time.sleep(2.0)

    text = raw_input("Safe to rotate? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        base.move_right()


    print "Experiment Done!!"

if __name__ == '__main__':
    main()