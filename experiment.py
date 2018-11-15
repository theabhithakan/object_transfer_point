#!/usr/bin/env python
import os
import sys
import rospy
import time
import math
import pickle
import scipy.io as sio
import numpy as np
from std_msgs.msg import Float64MultiArray
import baxter_interface
from handover.msg import skeleton
import promp
import move_base
import head_wobbler
from reflex_msgs.msg import Command
import xdisplay_image

obs_realtime = skeleton
obs_sub = None

class HumanSaver:
    """docstring for HumanSaver"""
    def __init__(self, heightIn, arm_lengthIn):

        self.subject = 8

        self.height = heightIn
        self.arm_length = arm_lengthIn
        # self.P_rw = [0.0, 0.0, 0.0]

        self.saver = {}
        self.P_rw = []
        self.saver["stand_right"] = np.matrix([0.0, 0.0, 0.0, 0.0, 0.0])
        self.saver["stand_front"] = np.matrix([0.0, 0.0, 0.0, 0.0, 0.0])
        self.saver["stand_back"] = np.matrix([0.0, 0.0, 0.0, 0.0, 0.0])
        self.saver["stand_back_hand1"] = np.matrix([0.0, 0.0, 0.0, 0.0, 0.0])
        self.saver["stand_back_hand2"] = np.matrix([0.0, 0.0, 0.0, 0.0, 0.0])
        self.saver["sit_right"] = np.matrix([0.0, 0.0, 0.0, 0.0, 0.0])
        self.saver["sit_front"] = np.matrix([0.0, 0.0, 0.0, 0.0, 0.0])
        self.saver["sit_back"] = np.matrix([0.0, 0.0, 0.0, 0.0, 0.0])
        self.saver["sit_back_hand1"] = np.matrix([0.0, 0.0, 0.0, 0.0, 0.0])
        self.saver["sit_back_hand2"] = np.matrix([0.0, 0.0, 0.0, 0.0, 0.0])
        self.demo_human_wrist = np.matrix([0.0, 0.0, 0.0, 0.0])
        self.D = np.matrix([0.0, 0.0, 0.0, 0.0])
        # self.obs_sub = rospy.Subscriber("skeleton_data", skeleton, self.callback, queue_size = 1)

        # self.P_rs = np.array()
        # self.P_ls = np.array()
        # self.P_rw = np.array()
        # self.P_lw = np.array()
        # self.P_h = np.array()

    def callback(self, data):

        # self.P_rs = np.array([data.joints[0].x, data.joints[0].y, data.joints[0].z])
        # self.P_ls = np.array([data.joints[1].x, data.joints[1].y, data.joints[1].z])
        # self.P_rw = np.array([data.joints[2].x, data.joints[2].y, data.joints[2].z])
        # self.P_lw = np.array([data.joints[3].x, data.joints[3].y, data.joints[3].z])
        # self.P_h = np.array([data.joints[4].x, data.joints[4].y, data.joints[4].z])

        # print [data.joints[0].x, data.joints[0].y, data.joints[0].z]

        self.P_rs = np.matrix([data.joints[0].x, data.joints[0].y, data.joints[0].z, float(data.joints[0].stamp)])
        self.P_ls = np.matrix([data.joints[1].x, data.joints[1].y, data.joints[1].z, float(data.joints[1].stamp)])
        self.P_rw = np.matrix([data.joints[2].x, data.joints[2].y, data.joints[2].z, float(data.joints[2].stamp)])
        self.P_lw = np.matrix([data.joints[3].x, data.joints[3].y, data.joints[3].z, float(data.joints[3].stamp)])
        self.P_h = np.matrix([data.joints[4].x, data.joints[4].y, data.joints[4].z, float(data.joints[4].stamp)])

        self.D = np.append(self.D, self.P_rw, axis=0)
        self.D = self.D[(np.shape(self.D)[0] - 2):np.shape(self.D)[0],:]

    def traj_saver(self, condition):
        count = 0
        self.start=0
        while True:
            try:
                # if count==0:
                #     #Extracting wrist and shoulder positions of human and baxter
                #     self.otp_s = np.array([0.0,0.0,0.5])

                # # Human wrist position: current and previous
                # P_new = self.D[1, 0:3]
                # P_old = self.D[0, 0:3]

                # # Distance between current location and goal position
                # e_old = np.linalg.norm(P_old - self.otp_s)
                # e_new = np.linalg.norm(P_new - self.otp_s)

                # if (e_old - e_new) > 0.0001 or self.start==1:

                if count==0:
                    self.start_time = time.time()
                    t0 = self.D[0,3]
                human_wrist = np.concatenate((self.D[1,:3],np.matrix([self.D[1,3]-t0])), axis=1)
            
                # self.demo_baxter_joints = np.concatenate((self.demo_baxter_joints, self.b), axis=0)
                self.demo_human_wrist = np.concatenate((self.demo_human_wrist, human_wrist), axis=0)
                # self.demo_baxter_pos = np.concatenate((self.demo_baxter_pos, self.b_pos), axis=0)
                count += 1
                self.start=1

            except KeyboardInterrupt:
                break

        # self.i += 1
        # self.demo_human_wrist[:,3] = np.cumsum(np.squeeze(self.demo_human_wrist[:,3])).T
        # self.timer = time.time() - self.start_time

        # if self.i == 1:
        #     self.ndemos_baxter_joints = [self.demo_baxter_joints[1:,:]]
        #     self.ndemos_human_wrist = [self.demo_human_wrist[1:,:]]
        #     self.ndemos_baxter_pos = [self.demo_baxter_pos[1:,:]]
        #     # self.ndemos_time = [self.timer]
        # else:
        #     self.ndemos_baxter_joints.append(self.demo_baxter_joints[1:,:])
        #     self.ndemos_human_wrist.append(self.demo_human_wrist[1:,:])
        #     self.ndemos_baxter_pos.append(self.demo_baxter_pos[1:,:])
            # self.ndemos_time.append(self.timer)

        np.savetxt("src/handover/object_transfer_point/data/subjects/human_" + str(self.subject) + "/human_wrist_" + str(self.subject) + "_" + str(condition) + ".csv", self.demo_human_wrist, delimiter=",")
        # np.savetxt("src/handover/object_transfer_point/data/subjects/human_wrist" + str(self.i) + ".csv", self.demo_baxter_joints[1:], delimiter=",")
        # np.savetxt("src/handover/object_transfer_point/data/subjects/human_wrist" + str(self.i) + ".csv", self.demo_baxter_pos[1:,:], delimiter=",")

        sio.savemat("src/handover/object_transfer_point/data/subjects/human_" + str(self.subject) + "/human_wrist_" + str(self.subject) + "_" + str(condition) + ".mat",{'human_demo_data':self.demo_human_wrist})
        # sio.savemat('src/handover/object_transfer_point/data/subjects/human_wrist.mat',{'baxter_demo_data':self.ndemos_baxter_joints})
        # sio.savemat('src/handover/object_transfer_point/data/subjects/human_wrist.mat',{'baxter_demo_pos_data':self.ndemos_baxter_pos})
        # sio.savemat('src/handover/scripts/DataICRA19/demo_time.mat',{'time':self.ndemos_time})


    # def pointingDirection(self):
    #     angle = np.arctan2(np.linalg.norm(self.P_rs[1] - self.P_rw[1]), np.linalg.norm(self.P_rs[0:2:3] - self.P_rw[0:2:3]))
        
    #     return np.rad2deg(angle)

def main():
    rospy.init_node("OTP_Experiment", disable_signals=True)
    grasp_pub = rospy.Publisher("/right_hand/command", Command, queue_size=10)
    hs = HumanSaver(1.75,0.73)

    face = head_wobbler.Wobbler()
    base = move_base.MoveBase()
    xdisplay_image.send_image("NeutralNEGray.jpg")

    obs_sub = rospy.Subscriber("skeleton_data", skeleton, hs.callback, queue_size = 1)
    time.sleep(2.0)
    # hs.pointingDirection()
    
    stand_right = promp.ProMP("src/handover/object_transfer_point/data/stand_right")
    stand_front = promp.ProMP("src/handover/object_transfer_point/data/stand_front")
    stand_back = promp.ProMP("src/handover/object_transfer_point/data/stand_back")
    sit_right = promp.ProMP("src/handover/object_transfer_point/data/sit_right")
    sit_front = promp.ProMP("src/handover/object_transfer_point/data/sit_front")
    sit_back = promp.ProMP("src/handover/object_transfer_point/data/sit_back")

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
    # stand_right.reset_right_hand()
    
    xdisplay_image.send_image("NeutralNEGray.jpg")
    face.look_right()

    text = raw_input("Start 1st case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        xdisplay_image.send_image("NeutralNEGreen.jpg")
        hs.traj_saver("stand_right")
        hs.saver["stand_right"] = [hs.height, hs.arm_length, hs.P_rw[0, 0], hs.P_rw[0, 1], hs.P_rw[0, 2]]
        obs_realtime = [[hs.saver["stand_right"][2], hs.saver["stand_right"][3], hs.saver["stand_right"][4]]]
        stand_right.test_promp(obs_realtime)
        time.sleep(2.0)
        grasp_pub.publish(take)
        time.sleep(4.0)
        stand_right.safe_right_dist()
        stand_right.to_the_basket()
        time.sleep(2.0)
        grasp_pub.publish(drop)
        time.sleep(2.0)
        face.set_neutral()
        stand_right.reset_right_hand()

    text = raw_input("Safe to rotate? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        base.move_right()

    xdisplay_image.send_image("NeutralNEYellow.jpg")

    text = raw_input("Start 2nd case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        xdisplay_image.send_image("NeutralNEGreen.jpg")
        hs.traj_saver("stand_front")
        hs.saver["stand_front"] = [hs.height, hs.arm_length, hs.P_rw[0, 0], hs.P_rw[0, 1], hs.P_rw[0, 2]]
        obs_realtime = [[hs.saver["stand_front"][2], hs.saver["stand_front"][3], hs.saver["stand_front"][4]]]
        stand_front.test_promp(obs_realtime)
        time.sleep(2.0)
        grasp_pub.publish(take)
        time.sleep(4.0)
        stand_front.safe_front_dist()
        stand_front.to_the_basket()
        time.sleep(2.0)
        grasp_pub.publish(drop)
        time.sleep(2.0)
        stand_front.reset_right_hand()


    xdisplay_image.send_image("NeutralNERed.jpg")
    
    text = raw_input("Start 3rd case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        xdisplay_image.send_image("NeutralNEGreen.jpg")
        hs.traj_saver("stand_back")
        hs.saver["stand_back"] = [hs.height, hs.arm_length, hs.P_rw[0, 0], hs.P_rw[0, 1], hs.P_rw[0, 2]]
        hs.saver["stand_back_hand1"] = [hs.height, hs.arm_length, hs.P_rw[0, 0], hs.P_rw[0, 1], hs.P_rw[0, 2]]
        hs.saver["stand_back_hand2"] = [hs.height, hs.arm_length, hs.P_lw[0, 0], hs.P_lw[0, 1], hs.P_lw[0, 2]]
        obs_realtime = [[hs.saver["stand_back"][2], hs.saver["stand_back"][3], hs.saver["stand_back"][4]]]
        stand_back.test_promp(obs_realtime)
        time.sleep(2.0)
        grasp_pub.publish(take)
        time.sleep(2.0)
        stand_back.safe_front_dist()
        stand_back.to_the_basket()
        time.sleep(2.0)
        grasp_pub.publish(drop)
        time.sleep(2.0)
        stand_back.reset_right_hand()

    text = raw_input("Safe to rotate? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        base.move_left()


    xdisplay_image.send_image("NeutralNEGray.jpg")
    face.look_right()
    
    text = raw_input("Start 4th case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        xdisplay_image.send_image("NeutralNEGreen.jpg")
        hs.traj_saver("sit_right")
        hs.saver["sit_right"] = [hs.height, hs.arm_length, hs.P_rw[0, 0], hs.P_rw[0, 1], hs.P_rw[0, 2]]
        obs_realtime = [[hs.saver["sit_right"][2], hs.saver["sit_right"][3], hs.saver["sit_right"][4]]]
        sit_right.test_promp(obs_realtime)
        time.sleep(2.0)
        grasp_pub.publish(take)
        time.sleep(2.0)
        sit_right.safe_right_dist()
        sit_right.to_the_basket()
        time.sleep(2.0)
        grasp_pub.publish(drop)
        time.sleep(2.0)
        face.set_neutral()
        sit_right.reset_right_hand()
    
    text = raw_input("Safe to rotate? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        base.move_right()



    xdisplay_image.send_image("NeutralNEYellow.jpg")
    
    text = raw_input("Start 5th case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        xdisplay_image.send_image("NeutralNEGreen.jpg")
        hs.traj_saver("sit_front")
        hs.saver["sit_front"] = [hs.height, hs.arm_length, hs.P_rw[0, 0], hs.P_rw[0, 1], hs.P_rw[0, 2]]
        obs_realtime = [[hs.saver["sit_front"][2], hs.saver["sit_front"][3], hs.saver["sit_front"][4]]]
        sit_front.test_promp(obs_realtime)
        time.sleep(2.0)
        grasp_pub.publish(take)
        time.sleep(2.0)
        sit_front.safe_front_dist()
        sit_front.to_the_basket()
        time.sleep(2.0)
        grasp_pub.publish(drop)
        time.sleep(2.0)
        sit_front.reset_right_hand()
    

    xdisplay_image.send_image("NeutralNERed.jpg")
    
    text = raw_input("Start 6th case? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        xdisplay_image.send_image("NeutralNEGreen.jpg")
        hs.traj_saver("sit_back")
        hs.saver["sit_back"] = [hs.height, hs.arm_length, hs.P_rw[0, 0], hs.P_rw[0, 1], hs.P_rw[0, 2]]
        hs.saver["sit_back_hand1"] = [hs.height, hs.arm_length, hs.P_rw[0, 0], hs.P_rw[0, 1], hs.P_rw[0, 2]]
        hs.saver["sit_back_hand2"] = [hs.height, hs.arm_length, hs.P_lw[0, 0], hs.P_lw[0, 1], hs.P_lw[0, 2]]
        obs_realtime = [[hs.saver["sit_back"][2], hs.saver["sit_back"][3], hs.saver["sit_back"][4]]]
        sit_back.test_promp(obs_realtime)
        time.sleep(2.0)
        grasp_pub.publish(take)
        time.sleep(2.0)
        sit_back.safe_front_dist()
        sit_back.to_the_basket()
        time.sleep(2.0)
        grasp_pub.publish(drop)
        time.sleep(2.0)
        sit_back.reset_right_hand()

    with open("src/handover/object_transfer_point/data/subjects/human_" + str(hs.subject) + "/human_wrist_" + str(hs.subject), "wb") as f:
        pickle.dump(hs.saver, f)
    sio.savemat("src/handover/object_transfer_point/data/subjects/human_" + str(hs.subject) + "/human_wrist_" + str(hs.subject) + ".mat", {'D':hs.saver})

    xdisplay_image.send_image("JoyNEOrange.jpg")

    text = raw_input("Safe to rotate? (y/n)")
    if text == 'n':
        print "ok bye"
    else:
        base.move_left()

    print "Experiment Done!!"

if __name__ == '__main__':
    main()