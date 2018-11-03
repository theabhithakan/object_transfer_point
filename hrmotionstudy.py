#!/usr/bin/env python
import sys, time
import rospy
import pickle
import numpy as np
import baxter_interface
import scipy.io as sio
from handover.msg import skeleton

class MotionStudy:

    def __init__(self):

        #Subscribing to the skeleton data from kinect
        self.data_sub = rospy.Subscriber("skeleton_data", skeleton, self.callback, queue_size = 1)
        
        # Initialize
        self.i = 0
        self.subject = 20
        self.human = {}
        self.human['rw'] = np.matrix([0.0, 0.0, 0.0, 0.0])
        self.human['rs'] = np.matrix([0.0, 0.0, 0.0, 0.0])
        self.human['lw'] = np.matrix([0.0, 0.0, 0.0, 0.0])
        self.human['ls'] = np.matrix([0.0, 0.0, 0.0, 0.0])
        self.human['h'] = np.matrix([0.0, 0.0, 0.0, 0.0])
        self.human['wh'] = np.matrix([[0.0],[0.8]])
        self.human['sh'] = np.matrix([[0.0],[1.43]])
        self.human['hh'] = np.matrix([[0.0],[1.64]])
        self.human['al'] = np.matrix([[0.0],[0.61]])

        # Move Baxter to Initial Position
        self.final_angles = {'right_s0': -0.22281070944035636, 'right_s1': 0.13345632854603098, 'right_w0': 0.8245146734884099, 'right_w1': 1.0473253829287663, 'right_w2': 0.39193209130472323, 'right_e0': 1.2893108522176902, 'right_e1': 1.7050196457346374}
        self.init_angles_right = {'right_s0': -1.469170099597255, 'right_s1': 0.24160197409195266, 'right_w0': -0.2807184841830307, 'right_w1': 0.8364030245945219, 'right_w2': 0.41724277430483253, 'right_e0': 0.5069806503961293, 'right_e1': 1.5500875861582106}
        init_angles_left = {'left_w0': -0.06864564025787226, 'left_w1': 1.0051409112619174, 'left_w2': -0.21015536794030168, 'left_e0': -0.4410194765170565, 'left_e1': 1.458432234082057, 'left_s0': 1.477606993930625, 'left_s1': 0.28455343615274425}
        self.righty = baxter_interface.Limb('right')
        lefty = baxter_interface.Limb('left')

        self.righty.move_to_joint_positions(self.init_angles_right,5,0.0015)
        lefty.move_to_joint_positions(init_angles_left,5,0.0015)

        # Start Recording
        text = raw_input("Start recording demo? (Y/n)")
        if text  == 'n':
            print "No demo recorded"
        else:
            self.start_time = time.time()
            self.rec()


    def callback(self,data):

        self.data = data
    
        #Sorting skeleton data
        self.P_rs = np.matrix([self.data.joints[0].x, self.data.joints[0].y, self.data.joints[0].z, float(self.data.joints[0].stamp)])
        self.P_rw = np.matrix([self.data.joints[2].x, self.data.joints[2].y, self.data.joints[2].z, float(self.data.joints[2].stamp)])
        self.P_ls = np.matrix([self.data.joints[1].x, self.data.joints[1].y, self.data.joints[1].z, float(self.data.joints[1].stamp)])
        self.P_lw = np.matrix([self.data.joints[3].x, self.data.joints[3].y, self.data.joints[3].z, float(self.data.joints[3].stamp)])
        self.P_h = np.matrix([self.data.joints[4].x, self.data.joints[4].y, self.data.joints[4].z, float(self.data.joints[4].stamp)])

    def rec(self):
        while True:
            try:
                self.human['rw'] = np.concatenate((self.human['rw'],self.P_rw), axis=0)
                self.human['rs'] = np.concatenate((self.human['rs'],self.P_rs), axis=0)
                self.human['lw'] = np.concatenate((self.human['lw'],self.P_lw), axis=0)
                self.human['ls'] = np.concatenate((self.human['ls'],self.P_ls), axis=0)
                self.human['h'] = np.concatenate((self.human['h'],self.P_h), axis=0)

            except KeyboardInterrupt:
                self.i = self.i + 1
                break
        
        thishuman = {}
        for key in self.human:
            thishuman[key] = self.human[key][1:,:]
        self.timer = time.time() - self.start_time

        print len(thishuman['rw'])

        if self.i == 1:
            self.nhuman = [thishuman]
            print type(self.nhuman)
            print len(self.nhuman[0]['rw'])
            self.ndemos_time = [self.timer]
        else:
            self.nhuman.append(thishuman)
            print type(self.nhuman)
            print len(self.nhuman)
            print len(self.nhuman[0]['rw'])
            print len(self.nhuman[-1]['rw'])
            self.ndemos_time.append(self.timer)

        time.sleep(1)
        self.righty.move_to_joint_positions(self.init_angles_right,5,0.003)

        text = raw_input("\nRecord another demo? (Y/n)")
        if text == 'n':
            print "Only", self.i, "demo(s) recorded"
        else:
            self.start_time = time.time()
            self.human['rw'] = np.matrix([0.0, 0.0, 0.0, 0.0])
            self.human['rs'] = np.matrix([0.0, 0.0, 0.0, 0.0])
            self.human['lw'] = np.matrix([0.0, 0.0, 0.0, 0.0])
            self.human['ls'] = np.matrix([0.0, 0.0, 0.0, 0.0])
            self.human['h'] = np.matrix([0.0, 0.0, 0.0, 0.0])
            print len(self.nhuman[0]['rw'])
            self.rec()

        with open("src/handover/scripts/DataMS/human" + str(self.subject),"wb") as f:
            pickle.dump(self.nhuman, f)
        sio.savemat("src/handover/scripts/DataMS/demo_time.mat",{'time':self.ndemos_time})
        sio.savemat("src/handover/scripts/DataMS/human" + str(self.subject) + ".mat",{'D':self.nhuman})

def main(args):
    rospy.init_node('MotionStudy', anonymous=True, disable_signals=True)
    ms = MotionStudy()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")


if __name__ == '__main__':
    main(sys.argv)