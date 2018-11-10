#!/usr/bin/env python
import sys
import rospy
import numpy as np
import baxter_interface
import scipy.io as sio
from handover.msg import skeleton
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64MultiArray
import time


class Saver:

    def __init__(self):
        
        # Initialize
        self.i = 0
        self.demo_baxter_joints = np.matrix([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.demo_baxter_pos = np.matrix([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        self.demo_human_wrist = np.matrix([0.0, 0.0, 0.0, 0.0])
        self.D = np.matrix([0.0, 0.0, 0.0, 0.0])
        self.right_limb = baxter_interface.Limb('right')
        self.left_limb = baxter_interface.Limb('left')
        self.start = 0

        left_angles = {'left_w0': -0.38502917775923884, 'left_w1': 0.06212622190935926, 'left_w2': -1.685077895492127, 'left_e0': 0.019558255045539024, 'left_e1': 1.2486603613387268, 'left_s0': 0.7589369948063085, 'left_s1': 0.3129320807286244}
        self.left_limb.move_to_joint_positions(left_angles,5)

        init_angles = {'right_s0': -0.7815632114276183, 'right_s1': 0.2519563444101792, 'right_w0': 1.2072428800658206, 'right_w1': 0.20862138715241627, 'right_w2': 0.5437961893053792, 'right_e0': -0.16528642989465334, 'right_e1': 1.2448254093690132}
        self.right_limb.move_to_joint_positions(init_angles,5)

        # Subscribing to the skeleton data from kinect
        self.data_sub = rospy.Subscriber("/skeleton_data", skeleton, self.callback1, queue_size = 1)
        # Subscribing to object grasp points data
        # self.data_sub = rospy.Subscriber("/objects/3d", PoseArray, self.callback2, queue_size = 1)

        # Start Recording
        text = raw_input("Start recording demo? (Y/n)")
        if text  == 'n':
            print "No demo recorded"
        else:
            # self.start_time = time.time()
            self.rec()

    def callback1(self,data):
        self.data = data
    
        #Sorting skeleton data
        P_rs = np.matrix([self.data.joints[0].x, self.data.joints[0].y, self.data.joints[0].z, float(self.data.joints[0].stamp)])
        self.P_rw = np.matrix([self.data.joints[2].x, self.data.joints[2].y, self.data.joints[2].z, float(self.data.joints[2].stamp)])
        P_ls = np.matrix([self.data.joints[1].x, self.data.joints[1].y, self.data.joints[1].z, float(self.data.joints[1].stamp)])
        P_lw = np.matrix([self.data.joints[3].x, self.data.joints[3].y, self.data.joints[3].z, float(self.data.joints[3].stamp)])
        P_h = np.matrix([self.data.joints[4].x, self.data.joints[4].y, self.data.joints[4].z, float(self.data.joints[4].stamp)])

        #User-Adaptive Frame
        P_rs = np.array([self.data.joints[0].x, self.data.joints[0].y, self.data.joints[0].z])
        P_ls = np.array([self.data.joints[1].x, self.data.joints[1].y, self.data.joints[1].z])
        P_o = (P_rs + P_ls)/2
        # theta = np.pi - np.arctan2((P_rs[2] - P_ls[2]),(P_rs[0] - P_ls[0]))
        # # print np.rad2deg(theta)
        # self.tf_k2h = np.array([[np.cos(theta), 0, -np.sin(theta), P_o[0]], [0, 1, 0, P_o[1]], [np.sin(theta), 0, np.cos(theta), P_o[2]], [0, 0, 0, 1]])
        # self.tf_h2k = np.linalg.inv(self.tf_k2h)
        # self.otp_point = np.matrix([self.data.joints[2].x, self.data.joints[2].y, self.data.joints[2].z, 1.0])
        # self.otp_point = np.matmul(self.tf_k2h, self.otp_point.T)
        # self.otp_point[3,0] = self.P_rw[0,3]
        # self.P_rw = self.otp_point.T

        #Formatting the sensor data
        self.D = np.append(self.D, self.P_rw, axis=0)
        print self.D.shape
        self.D = self.D[(np.shape(self.D)[0] - 2):np.shape(self.D)[0],:] 
        # print self.D.shape

        #Saving variables
        baxarm_angles = self.right_limb.joint_angles()
        self.b = np.matrix([baxarm_angles['right_s0'], baxarm_angles['right_s1'], baxarm_angles['right_e0'], baxarm_angles['right_e1'], baxarm_angles['right_w0'], baxarm_angles['right_w1'], baxarm_angles['right_w2']])
        baxarm_pos = self.right_limb.endpoint_pose()
        self.b_pos = np.matrix([baxarm_pos['position'].x, baxarm_pos['position'].y, baxarm_pos['position'].z, baxarm_pos['orientation'].x, baxarm_pos['orientation'].y, baxarm_pos['orientation'].z, baxarm_pos['orientation'].w])

        # print "callback"

    def rec(self):
        count = 0
        while True:
            try:
                # if count==0:
                #     #Extracting wrist and shoulder positions of human and baxter
                #     self.otp_s = np.array([0.0,0.0,0.5])

                #Human wrist position: current and previous
                # P_new = self.D[1, 0:3]
                # P_old = self.D[0, 0:3]

                #Distance between current location and goal position
                # e_old = np.linalg.norm(P_old - self.otp_s)
                # e_new = np.linalg.norm(P_new - self.otp_s)

                # if (e_old - e_new) > 0.0001 or self.start==1:
                self.start = 1
                print "moving"
                if count==0:
                    self.start_time = time.time()
                    t0 = self.D[0,3]
                human_wrist = np.concatenate((self.D[1,:3],np.matrix([self.D[1,3]-t0])), axis=1)
            
                self.demo_baxter_joints = np.concatenate((self.demo_baxter_joints, self.b), axis=0)
                self.demo_human_wrist = np.concatenate((self.demo_human_wrist, human_wrist), axis=0)
                self.demo_baxter_pos = np.concatenate((self.demo_baxter_pos, self.b_pos), axis=0)
                count += 1

            except KeyboardInterrupt:
                break

        self.i += 1
        # self.demo_human_wrist[:,3] = np.cumsum(np.squeeze(self.demo_human_wrist[:,3])).T
        # self.timer = time.time() - self.start_time

        if self.i == 1:
            self.ndemos_baxter_joints = [self.demo_baxter_joints[1:,:]]
            self.ndemos_human_wrist = [self.demo_human_wrist[1:,:]]
            self.ndemos_baxter_pos = [self.demo_baxter_pos[1:,:]]
            # self.ndemos_time = [self.timer]
        else:
            self.ndemos_baxter_joints.append(self.demo_baxter_joints[1:,:])
            self.ndemos_human_wrist.append(self.demo_human_wrist[1:,:])
            self.ndemos_baxter_pos.append(self.demo_baxter_pos[1:,:])
            # self.ndemos_time.append(self.timer)

        np.savetxt("src/handover/object_transfer_point/data/demo_baxter_" + str(self.i) + ".csv", self.demo_baxter_joints[1:], delimiter=",")
        np.savetxt("src/handover/object_transfer_point/data/demo_human_" + str(self.i) + ".csv", self.demo_human_wrist[1:,:], delimiter=",")
        np.savetxt("src/handover/object_transfer_point/data/demo_pos_" + str(self.i) + ".csv", self.demo_baxter_pos[1:,:], delimiter=",")

        time.sleep(1)

        init_angles = {'right_s0': -0.7815632114276183, 'right_s1': 0.2519563444101792, 'right_w0': 1.2072428800658206, 'right_w1': 0.20862138715241627, 'right_w2': 0.5437961893053792, 'right_e0': -0.16528642989465334, 'right_e1': 1.2448254093690132}
        self.right_limb.move_to_joint_positions(init_angles,3)

        text = raw_input("\nRecord another demo? (Y/n)")
        if text == 'n':
            print "Only", self.i, "demo(s) recorded"
        else:
            self.demo_baxter_joints = np.matrix([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.demo_baxter_pos = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
            self.demo_human_wrist = np.matrix([0.0, 0.0, 0.0, 0.0])
            # self.start_time = time.time()
            self.rec()

        sio.savemat('src/handover/object_transfer_point/data/demo_baxter.mat',{'baxter_demo_data':self.ndemos_baxter_joints})
        sio.savemat('src/handover/object_transfer_point/data/demo_baxter_pos.mat',{'baxter_demo_pos_data':self.ndemos_baxter_pos})
        sio.savemat('src/handover/object_transfer_point/data/demo_human.mat',{'human_demo_data':self.ndemos_human_wrist})
        # sio.savemat('src/handover/scripts/DataICRA19/demo_time.mat',{'time':self.ndemos_time})
        
    def callback2(self,data):
        ps = data

        self.h = np.matrix([ps.poses[0].position.x,ps.poses[0].position.y,ps.poses[0].position.z,ps.poses[1].position.x,ps.poses[1].position.y,ps.poses[1].position.z,ps.poses[2].position.x,ps.poses[2].position.y,ps.poses[2].position.z])

        baxarm_angles = self.right_limb.joint_angles()
        baxarm_endpose = self.right_limb.endpoint_pose()
        self.b = Float64MultiArray()
        self.b.data = np.matrix([baxarm_angles['right_s0'], baxarm_angles['right_s1'], baxarm_angles['right_e0'], baxarm_angles['right_e1'], baxarm_angles['right_w0'], baxarm_angles['right_w1'], baxarm_angles['right_w2']])
        baxarm_pos = self.right_limb.endpoint_pose()
        self.baxarm_pos = np.array([baxarm_pos['position'].x, baxarm_pos['position'].y, baxarm_pos['position'].z, baxarm_pos['orientation'].x, baxarm_pos['orientation'].y, baxarm_pos['orientation'].z, baxarm_pos['orientation'].w])
        
    def save(self):
        while True:
            try:
                self.demo_baxter_joints = np.concatenate((self.demo_baxter_joints, self.b.data), axis=0)
                self.demo_human_wrist = np.concatenate((self.demo_human_wrist,self.h), axis=0)
                self.demo_baxter_pos = np.concatenate((self.demo_baxter_pos, [self.baxarm_pos]), axis=0)


            except KeyboardInterrupt:
                self.i = self.i + 1
                break

        self.timer = time.time() - self.start_time

        if self.i == 1:
            self.ndemos_baxter_joints = [self.demo_baxter_joints[1:,:]]
            self.ndemos_human_wrist = [self.demo_human_wrist[1:,:]]
            self.ndemos_baxter_pos = [self.demo_baxter_pos[1:,:]]
            self.ndemos_time = [self.timer]
        else:
            self.ndemos_baxter_joints.append(self.demo_baxter_joints[1:,:])
            self.ndemos_human_wrist.append(self.demo_human_wrist[1:,:])
            self.ndemos_baxter_pos.append(self.demo_baxter_pos[1:,:])
            self.ndemos_time.append(self.timer)

        np.savetxt("src/handover/object_transfer_point/data/stand_right/demo_baxter_" + str(self.i) + ".csv", self.demo_baxter_joints[1:], delimiter=",")
        np.savetxt("src/handover/object_transfer_point/data/stand_right/demo_human_" + str(self.i) + ".csv", self.demo_human_wrist[1:,:], delimiter=",")
        np.savetxt("src/handover/object_transfer_point/data/stand_right/demo_pos_" + str(self.i) + ".csv", self.demo_baxter_pos[1:,:], delimiter=",")

        time.sleep(1)

        init_angles = {'right_s0': -1.6889128474618404, 'right_s1': 0.6412039693361029, 'right_w0': -0.1392087565006013, 'right_w1': 0.006519418348513008, 'right_w2': -3.0464858447404315, 'right_e0': 0.5829126993964572, 'right_e1': 1.0737865515197895}    
        self.right_limb.move_to_joint_positions(init_angles)

        text = raw_input("\nRecord another demo? (Y/n)")
        if text == 'n':
            print "Only", self.i, "demo(s) recorded"
        else:
            self.demo_baxter_joints = np.matrix([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.demo_baxter_pos = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
            self.demo_human_wrist = np.matrix([0.0, 0.0, 0.0, 0.0, 0.0])
            self.start_time = time.time()
            self.rec()

        sio.savemat('src/handover/object_transfer_point/data/stand_right/demo_baxter.mat',{'baxter_demo_data':self.ndemos_baxter_joints})
        sio.savemat('src/handover/object_transfer_point/data/stand_right/demo_baxter_pos.mat',{'baxter_demo_pos_data':self.ndemos_baxter_pos})
        sio.savemat('src/handover/object_transfer_point/data/stand_right/demo_human.mat',{'human_demo_data':self.ndemos_human_wrist})
        sio.savemat('src/handover/object_transfer_point/data/stand_right/demo_time.mat',{'time':self.ndemos_time})

def main(args):
    rospy.init_node('Saver', anonymous=True, disable_signals=True)
    sav = Saver()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")


if __name__ == '__main__':
    main(sys.argv)