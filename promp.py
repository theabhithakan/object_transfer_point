#!/usr/bin/env python
import pdb
import rospy
import sys, time, os
import numpy as np
from scipy import linalg
# from handover.msg import skeleton
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseArray
from reflex_msgs.msg import Command
import baxter_interface
from scipy.optimize import minimize
from scipy.spatial.distance import mahalanobis

class ProMP:

    '''
    ------------------------------------------------------PROMP INITIALIZATION-----------------------------------------------------
    '''
    
    def __init__(self, training_address):
        self.goal_pub = rospy.Publisher("/promp_data", Float64MultiArray, queue_size=1)
        self.grasp_pub = rospy.Publisher("/right_hand/command", Command, queue_size=10)

        self.ndemos = 17
        self.obs_dofs = 4
        self.bax_dofs = 7
        self.stdev = 0.005
        self.D = np.matrix([0.0, 0.0, 0.0, 0.0])
        # self.count = 0
        # self.phase_z = 0
        self.dt = 0.01
        # self.start = 0

        self.p_data, self.q_data = self.loadData(self.dt, training_address)
        self.promp = self.pmpRegression(self.q_data)

        self.param = {"nTraj":self.p_data["size"], "nTotalJoints":self.promp["nJoints"], "observedJointPos":np.array([0,1,2,3]), "observedJointVel":np.array([])}
        print "Initialized"
        
        # Subscribe to observed data
        # self.obs_sub = rospy.Subscriber("skeleton_data", skeleton, self.callback, queue_size = 1)
        # self.obs_sub = rospy.Subscriber("/objects/3d", PoseArray, self.callback, queue_size=1)

        self.limb = baxter_interface.Limb('right')
        # init_angles = {'right_s0': -1.469170099597255, 'right_s1': 0.24160197409195266, 'right_w0': -0.2807184841830307, 'right_w1': 0.8364030245945219, 'right_w2': 0.41724277430483253, 'right_e0': 0.5069806503961293, 'right_e1': 1.5500875861582106}
        # self.limb.move_to_joint_positions(init_angles,timeout=4.0)
        # print "Trained"

        # text = raw_input("Run obstacle avoidance? (Y/n)")
        # if text  == 'n':
        #     print "Hmm Adventurous..."
        # else:
        #     self.obstacleAvoidance()

        # grip = Command()
        # grip.pose.f1 = 0.0
        # grip.pose.f2 = 0.0
        # grip.pose.f3 = -2.0
        # grip.pose.preshape = 0.0
        # grip.velocity.f1 = 2.0
        # grip.velocity.f2 = 2.0
        # grip.velocity.f3 = 2.0
        # grip.velocity.preshape = 2.0
        # self.grasp_pub.publish(grip)

        # text = raw_input("Run ProMP? (Y/n)")
        # if text  == 'n':
        #     print "Alright. Not moving."
        # else:
        #     time.sleep(1)
        #     print "GOOOO!"
        #     self.start = 1

    def reset_right_hand(self):
        reset_angles = {'right_s0': -0.1902136176977913, 'right_s1': -0.24236896448589537, 'right_w0': 1.6171992456281974, 'right_w1': 0.4966262800779027, 'right_w2': -2.9931800123614134, 'right_e0': 0.9583544972314122, 'right_e1': 1.2133788032173622}
        reset_angles = init_angles = {'right_s0': -1.469170099597255, 'right_s1': 0.24160197409195266, 'right_w0': -0.2807184841830307, 'right_w1': 0.8364030245945219, 'right_w2': 0.41724277430483253, 'right_e0': 0.5069806503961293, 'right_e1': 1.5500875861582106}
        self.limb.move_to_joint_positions(reset_angles,timeout=4.0)


    def test_promp(self):
        '''
        Test ProMP
        '''
        time.sleep(2)
        self.this_phase = 40
        self.obs_pose = self.q_data[2][self.this_phase,:3]
        print self.obs_pose
        self.runPromp()

    def callback(self,data):
        # pos = data.poses
        # point = np.array([pos[0].position.x,pos[0].position.y,pos[0].position.z])
        # obs = self.Observation(self.stdev,self.param,self.p_data,point)
        # self.kf = self.kfLoop(self.promp,self.param,obs)
        
        # self.P_rw = np.matrix([data.joints[2].x, data.joints[2].y, data.joints[2].z, float(data.joints[2].stamp)])

        #User-Adaptive Frame
        P_rs = np.array([data.joints[0].x, data.joints[0].y, data.joints[0].z])
        P_ls = np.array([data.joints[1].x, data.joints[1].y, data.joints[1].z])
        # P_o = (P_rs + P_ls)/2
        # theta = np.pi - np.arctan2((P_rs[2] - P_ls[2]),(P_rs[0] - P_ls[0]))
        # # print np.rad2deg(theta)
        # self.tf_k2h = np.array([[np.cos(theta), 0, -np.sin(theta), P_o[0]], [0, 1, 0, P_o[1]], [np.sin(theta), 0, np.cos(theta), P_o[2]], [0, 0, 0, 1]])
        # self.tf_h2k = np.linalg.inv(self.tf_k2h)
        # self.otp_point = np.matrix([data.joints[2].x, data.joints[2].y, data.joints[2].z, 1.0])
        # self.otp_point = np.matmul(self.tf_k2h, self.otp_point.T)
        # self.otp_point[3,0] = self.P_rw[0,3]
        # self.P_rw = self.otp_point.T

        self.D = np.append(self.D, self.P_rw, axis=0)
        self.D = self.D[(np.shape(self.D)[0] - 2):np.shape(self.D)[0],:]

        #Extracting wrist and shoulder positions of human and baxter
        self.otp_s = (P_rs+P_ls)/2
        self.otp_s[2] = self.otp_s[2]/2

        #Human wrist position: current and previous
        P_new = self.D[1, 0:3]
        P_old = self.D[0, 0:3]

        #Distance between current location and goal position
        e_old = np.linalg.norm(P_old - self.otp_s)
        e_new = np.linalg.norm(P_new - self.otp_s)
        
        if (((e_old - e_new) > 0.001) and self.start == 1) or self.phase_z > 30:
            
            if self.count==0:
                self.t0 = self.D[0,3]
                self.timer_begin = time.time()
                self.count += 1
            self.phase_t  = self.D[1,3]-self.t0

            '''
            Phase Estimation
            '''
            t_sum = np.zeros((100,len(self.q_data)))
            for i in range(0,len(self.q_data)):
                t_sum[:,i] = self.q_data[i][:,3].T
            t_mean = t_sum.sum(axis=1)/self.ndemos

            self.phase_z = (np.abs(t_mean - self.phase_t)).argmin()
            print self.phase_z

        # if 1 <= self.phase_z and self.phase_z < 50:
        #     self.moveBaxter()   

        if 10 <= self.phase_z and self.phase_z <= 60:
            self.this_phase = self.phase_z
            print self.phase_z
            self.runPromp()

        if self.phase_z > 90:
            error = self.D[1,0:3] - self.hand
            error = np.linalg.norm(error)
            # print 'error is', error

    def moveBaxter(self):
        final_angles = {'right_s0': -0.09127185687918211, 'right_s1': 0.018407769454624964, 'right_w0': 0.9311263382464462, 'right_w1': 1.1083011192472114, 'right_w2': 0.5606699779721187, 'right_e0': 1.2778059963085497, 'right_e1': 1.45076233014263}
        self.limb.set_joint_positions(final_angles)
        self.limb.move_to_joint_positions(final_angles,timeout=2.75,threshold=0.05) 
        # elapsed_time = time.time() - self.timer_begin
        # print 'time is', elapsed_time
        self.hand = np.matrix([0.06,0.002,0.57])

    def obstacleAvoidance(self):
        t_o = 70
        nJoints = self.promp["w"]["nJoints"]
        w0, b = [], []
        for i in range(nJoints):
            w0.append(self.kf["w_mean"][i])
            b.append(self.kf["q_sigma_ii"][i][t_o])
        self.w0 = np.matrix(np.array(w0)).T
        self.b_std = np.matrix(np.array(b))
        self.base = self.promp["basis"]["Gn"][t_o]

        bnd = (-30,30)
        Bnd = (bnd,)*300
        con1 = {'type': 'eq', 'fun': self.constraint1}
        con2 = {'type': 'ineq', 'fun': self.constraint2}
        cons = [con1,con2]

        sol = minimize(self.objective, w0, method='SLSQP', bounds=Bnd, constraints=cons)

        w = sol["x"]
        w = np.matrix([w[:30],w[30:60],w[60:90],w[90:120],w[120:150],w[150:180],w[180:210],w[210:240],w[240:270],w[270:300]])
        y_new = w*self.promp["basis"]["Gn"].T

        # for i in range(nJoints):
        #     self.kf["q_mean"][i] = y_new[i]

        traj_plot = np.empty((0,7),dtype=float)
        for j in range(y_new.shape[1]):
            promp = []
            for i in range(y_new.shape[0]):
                    promp.append(y_new[i,j])
            promp = np.array(promp[3:10])
            print promp.shape
            print np.matrix(promp).shape

            traj_plot = np.append(traj_plot,np.matrix(promp),axis=0)

        np.savetxt("../scripts/traj1.csv", traj_plot, delimiter=",")

    def objective(self,w):
        w = np.matrix([w[:30],w[30:60],w[60:90],w[90:120],w[120:150],w[150:180],w[180:210],w[210:240],w[240:270],w[270:300]]).T
        # o = np.matmul((w-w_mean).T,(w-w_mean))
        o = np.linalg.norm(w - self.w0)**2
        return o

    def constraint1(self,w):
        y_o = np.array([0.6,-0.5,0.2])
        D = np.array([0.15,0.15,0.15])
        w = np.matrix([w[:30],w[30:60],w[60:90],w[90:120],w[120:150],w[150:180],w[180:210],w[210:240],w[240:270],w[270:300]])
        # c1 = ((y - basis.T*w).T * M * (y - basis.T*w)) - D**2
        y_w = w*self.base.T
        c1 = np.linalg.norm(y_o[0:3] - y_w[0:3] - D)**2
        return c1

    def constraint2(self,w):
        w = np.matrix([w[:30],w[30:60],w[60:90],w[90:120],w[120:150],w[150:180],w[180:210],w[210:240],w[240:270],w[270:300]])
        x = w*self.base.T
        y = self.base*self.w0
        z = np.linalg.inv(np.cov(self.w0.T))
        mb = mahalanobis(x,y,z)
        c2 = np.linalg.norm(self.b_std - mb)**2
        return c2

    def runPromp(self):

        '''
        ProMP Estimation
        '''
        # self.obs_pose = self.D[1,:3]
        param = {"nTraj":self.p_data["size"], "nTotalJoints":self.promp["nJoints"], "observedJointPos":np.array([0,1,2]), "observedJointVel":np.array([])}
        obs = self.Observation(self.stdev,param,self.p_data,self.obs_pose,[self.this_phase])
        self.kf = self.kfLoop(self.promp,param,obs)

        print "sliced"

        ''' 
        OBJECT GRASPING CODE
        '''
        # grip = Command()
        # grip.pose.f1 = 0.0
        # grip.pose.f2 = 0.0
        # grip.pose.f3 = 0.0
        # grip.pose.preshape = 0.0
        # grip.velocity.f1 = 2.0
        # grip.velocity.f2 = 2.0
        # grip.velocity.f3 = 2.0
        # grip.velocity.preshape = 2.0
        # self.grasp_pub.publish(grip)
        
        # r = rospy.Rate(7)
        # traj_plot = np.empty((0,7),dtype=float)
        # for j in range(len(self.kf["q_mean"][3])):
        #     promp = []
        #     for i in range(len(self.kf["q_mean"])):
        #             promp.append(self.kf["q_mean"][i][j])
        #     promp = np.array(promp[3:10])

        #     traj_plot = np.append(traj_plot,np.matrix(promp),axis=0)

        #     P = Float64MultiArray()
        #     P.data = np.array(promp)
        #     self.goal_pub.publish(P)
        #     r.sleep()

        # np.savetxt("../scripts/traj2.csv", traj_plot, delimiter=",")

        # t = time.time()
        # tt = 0
        # r2 = rospy.Rate(7)
        # while tt < 1:
        #     self.goal_pub.publish(P)
        #     tt = time.time() - t
        #     r2.sleep()


        # text = raw_input("Grab bottle? (Y/n)")
        # if text  == 'n':
        #     print "Ok. Going back."
        # else:
        #     grip.pose.f1 = 2.0
        #     grip.pose.f2 = 2.0
        #     grip.pose.f3 = 2.0
        #     grip.pose.preshape = 0.0
        #     grip.velocity.f1 = 2.0
        #     grip.velocity.f2 = 2.0
        #     grip.velocity.f3 = 2.0
        #     grip.velocity.preshape = 2.0

        #     self.grasp_pub.publish(grip)
        #     time.sleep(2.0)

        #     text = raw_input("Give to human? (Y/n)")
        #     if text  == 'n':
        #         print "Ok. Going back."
        #     else:
        #         otp = {'right_s0': 0.6507913492603867, 'right_s1': -0.19941750242510378, 'right_w0': 1.992257548266181, 'right_w1': 0.043334957257762936, 'right_w2': -3.008136325043296, 'right_e0': 1.0496263541105944, 'right_e1': 0.4855049193657335}
        #         self.limb.move_to_joint_positions(otp,timeout=2.0)

        #         grip.pose.f1 = 0.0
        #         grip.pose.f2 = 0.0
        #         grip.pose.f3 = 0.0
        #         grip.pose.preshape = 0.0
        #         grip.velocity.f1 = 2.0
        #         grip.velocity.f2 = 2.0
        #         grip.velocity.f3 = 2.0
        #         grip.velocity.preshape = 2.0

        #         self.grasp_pub.publish(grip)
        #         time.sleep(2.0)

        # goback = {'right_s0': -0.1902136176977913, 'right_s1': -0.24236896448589537, 'right_w0': 1.6171992456281974, 'right_w1': 0.4966262800779027, 'right_w2': -2.9931800123614134, 'right_e0': 0.9583544972314122, 'right_e1': 1.2133788032173622}
        # self.limb.move_to_joint_positions(goback,timeout=2.0)
    
        '''
        Populate data for publishing
        '''
        # promp = []
        # for i in range(len(self.kf["q_mean"])):
        #         promp.append(self.kf["q_mean"][i][98])
        # self.hand = np.array(promp[0:3])
        # promp = np.array(promp[4:11])

        # W = (((self.this_phase/100.0)-1)**3 + 1)
        # # self.off_otp = np.matrix([0.0505,0.0837,0.54])
        # print self.hand
        # self.obs_pose = W*self.hand+ (1-W)*self.otp_s #self.off_otp 
        # print self.obs_pose
        # param = {"nTraj":self.p_data["size"], "nTotalJoints":self.promp["nJoints"], "observedJointPos":np.array([0,1,2]), "observedJointVel":np.array([])}
        # obs = self.Observation(self.stdev,param,self.p_data,self.obs_pose,[98])
        # self.kf = self.kfLoop(self.promp,param,obs)

        promp = []
        for i in range(len(self.kf["q_mean"])):
                promp.append(self.kf["q_mean"][i][98])

        self.hand = np.array(promp[0:3])
        # print "otp_s is", self.otp_s 
        print "promp estimate is", self.hand
        promp = np.array(promp[4:11])

        # P = Float64MultiArray()
        # P.data = self.hand
        # self.goal_pub.publish(P)

        otp_angles = {'right_s0': promp[0], 'right_s1': promp[1], 'right_w0': promp[4], 'right_w1': promp[5], 'right_w2': promp[6], 'right_e0': promp[2], 'right_e1': promp[3]}
        
        self.limb.move_to_joint_positions(otp_angles,timeout=2.75,threshold=0.05)
        # self.limb.set_joint_positions(otp_angles)
        # if self.phase_z < 50:
        #     self.limb.set_joint_positions(otp_angles)
        #     grip = Command()
        #     grip.pose.f1 = 0.0
        #     grip.pose.f2 = 0.0
        #     grip.pose.f3 = 0.0
        #     grip.pose.preshape = 0.0
        #     grip.velocity.f1 = 2.0
        #     grip.velocity.f2 = 2.0
        #     grip.velocity.f3 = 2.0
        #     grip.velocity.preshape = 2.0
        #     self.grasp_pub.publish(grip)
        # # elif self.phase_z<90:
        # #     self.limb.set_joint_positions(otp_angles)
        # else:
        #     self.limb.move_to_joint_positions(otp_angles,timeout=2.75,threshold=0.05)
            # grip = Command()
            # grip.pose.f1 = 2.0
            # grip.pose.f2 = 2.0
            # grip.pose.f3 = 2.0
            # grip.pose.preshape = 0.0
            # grip.velocity.f1 = 2.0
            # grip.velocity.f2 = 2.0
            # grip.velocity.f3 = 2.0
            # grip.velocity.preshape = 2.0
            # self.grasp_pub.publish(grip)
            # time.sleep(2.0)

        # elapsed_time = time.time() - self.timer_begin
        # print 'time is', elapsed_time

        # self.goal_pub.publish(P)

        # text = raw_input("Restart motion? (Y/n)")
        # if text  == 'n':
        #     print "Ok. Not moving."
        # else:
        #     init_angles = {'right_s0': -1.6889128474618404, 'right_s1': 0.6412039693361029, 'right_w0': -0.1392087565006013, 'right_w1': 0.006519418348513008, 'right_w2': -3.0464858447404315, 'right_e0': 0.5829126993964572, 'right_e1': 1.0737865515197895}
        #     self.limb.move_to_joint_positions(init_angles,timeout=2.0)

        # text = raw_input("Run ProMP again? (Y/n)")
        # if text  == 'n':
        #     print "Ok. Not moving."
        # else:
        #     self.runPromp()

        '''
        GRASPING
        '''
        # if phase_z > 90:
        #     self.start = 0

        #     self.limb.move_to_joint_positions(init_angles)


    def loadData(self,dt,training_address):
        bax_data, obs_data = [], []
        for i in range(self.ndemos):
            bax = np.loadtxt(open(training_address + "/demo_baxter_"+ str(i+1) +".csv", "rb"), delimiter=",")
            bax_data.append(bax)
            obs = np.loadtxt(open(training_address + "/demo_human_"+ str(i+1) +".csv", "rb"), delimiter=",")
            obs_data.append(obs[:,:4])

        bax, bax_mean = self.addVelocity(bax_data)
        obs, obs_mean = self.addVelocity(obs_data)

        demo_data = []
        for i in range(len(bax)):
            demo_data.append(np.concatenate((obs[i],bax[i]),axis=1))
        demo_mean = np.concatenate((obs_mean,bax_mean),axis=1)

        demo = {"q":[], "qdot":[], "q_mean":[], "q_cov":[], "q_var":[], "qdot_mean":[], "qdot_cov":[], "qdot_var":[]}
        for i in range(demo_data[0].shape[1]/2):
            q = q_dot = []
            for j in range(len(demo_data)):
                q.append(demo_data[j][:,2*i].T)
                q_dot.append(demo_data[j][:,(2*i)+1].T)    #TODO: Velocity not recalculated like in MATLAB. VERIFY!!???                                       
            demo["q"].append(np.matrix(q))
            demo["qdot"].append(np.matrix(q_dot))

        for i in range(demo_data[0].shape[1]/2):
            demo["q_mean"].append(np.mean(demo["q"][i],axis=0))
            demo["q_cov"].append(np.cov(demo["q"][i].T))
            demo["q_var"].append(np.var(demo["q"][i],axis=0,ddof=1).T)
            demo["qdot_mean"].append(np.mean(demo["qdot"][i],axis=0))
            demo["qdot_cov"].append(np.cov(demo["qdot"][i].T))
            demo["qdot_var"].append(np.var(demo["qdot"][i],axis=0,ddof=1).T)

        demo["size"] = demo_data[0].shape[0]

        # print demo["qdot_mean"][4]
        # print demo["q"][0].shape, demo["qdot"][0].shape, demo["q_mean"][0].shape, demo["q_cov"][0].shape, demo["q_var"][0].shape, demo["qdot_mean"][0].shape, demo["qdot_cov"][0].shape, demo["qdot_var"][0].shape

        demo_q = []
        for i in range(len(demo_data)):
            q = []
            for j in range(demo_data[0].shape[1]/2):
                q.append(demo_data[i][:,2*j])
            demo_q.append(np.matrix(q).T)

        print "Data Loaded"
        return demo, demo_q

    def addVelocity(self,data):
        for k in range(len(data)):
            d = data[k]

            vel = np.zeros((d.shape[0],2*d.shape[1]))
            for i in range(d.shape[1]):
                vel[:,2*i] = d[:,i]
                for j in range(1,d.shape[0]):
                    vel[j,(2*i)+1] = (d[j,i] - d[j-1,i])/self.dt
            vel_d = np.zeros((100,vel.shape[1]))
            for i in range(vel.shape[1]):
                xo = np.linspace(0,99,100)
                xp = np.linspace(0,99,d.shape[0])
                vel_d[:,i] = np.interp(xo,xp,vel[:,i])
            data[k] = vel_d

        mean = sum(data)/len(data)

        return data, mean

    def pmpRegression(self,data,nBasis=30):
        nJoints = data[0].shape[1]
        nDemo = len(data)
        nTraj = data[0].shape[0]

        mu_location = np.linspace(0, 1, nBasis)
        phase = self.Phase(self.dt)

        weight = {"nBasis":nBasis, "nJoints":nJoints, "nTraj":nTraj, "nDemo":nDemo}
        weight["my_linRegRidgeFactor"] = 1e-08 * np.identity(nBasis)        

        sigma = 0.05 * np.ones((1, nBasis))

        basis = self.generateGaussianBasis(phase, mu_location, sigma)

        weight = self.leastSquareOnWeights(weight, basis["Gn"], data)

        pmp = {"phase":phase, "w":weight, "basis":basis, "nBasis":nBasis, "nJoints":nJoints, "nDemo":nDemo, "nTraj":nTraj}

        return pmp

    def Phase(self,t):
        phase = {"dt":t}
        phase["z"] = np.linspace(t,1,1/t)
        zd = np.diff(phase["z"])/t
        phase["zd"] = np.append(zd,zd[-1])
        zdd = np.diff(phase["zd"])/t
        phase["zdd"] = np.append(zdd,zdd[-1])

        return phase

    # def Weight(nBasis,nJoints,nTraj,nDemo):
    #     weight = {"nBasis":nBasis, "nJoints":nJoints, "nTraj":nTraj, "nDemo":nDemo}
    #     weight["my_linRegRidgeFactor"] = 1e-08 * np.ones((nBasis,nBasis))


    def generateGaussianBasis(self,phase,mu,sigma):
        basisCenter = mu
        z = phase["z"]
        zd = phase["zd"]
        zdd = phase["zdd"]

        z_minus_center = np.matrix(z).T - np.matrix(basisCenter)

        at = np.multiply(z_minus_center, (1.0/sigma))

        Basis = {}

        basis = np.multiply(np.exp(-0.5*np.power(at,2)), 1./sigma/np.sqrt(2*np.pi))
        basis_sum = np.sum(basis, axis = 1)
        basis_n = np.multiply(basis, 1.0/basis_sum)

        z_minus_center_sigma = np.multiply(-z_minus_center, 1.0/np.power(sigma,2))
        basisD = np.multiply(z_minus_center_sigma, basis)

        # normalizing basisD
        basisD_sum = np.sum(basisD, axis = 1)
        basisD_n_a = np.multiply(basisD, basis_sum)
        basisD_n_b = np.multiply(basis, basisD_sum)
        basisD_n = np.multiply(basisD_n_a - basisD_n_b, 1.0/np.power(basis_sum,2))

        # second derivative of the basis
        tmp = np.multiply(basis, -1.0/np.power(sigma,2))
        basisDD = tmp + np.multiply(z_minus_center_sigma, basisD)
        basisDD_sum = np.sum(basisDD, axis = 1)

        # normalizing basisDD
        basisDD_n_a = np.multiply(basisDD, np.power(basis_sum,2))
        basisDD_n_b1 = np.multiply(basisD, basis_sum)
        basisDD_n_b = np.multiply(basisDD_n_b1, basisD_sum)
        basisDD_n_c1 = 2 * np.power(basisD_sum,2) - np.multiply(basis_sum, basisDD_sum)
        basisDD_n_c = np.multiply(basis, basisDD_n_c1)
        basisDD_n_d = basisDD_n_a - 2 * basisDD_n_b + basisDD_n_c
        basisDD_n = np.multiply(basisDD_n_d, 1.0/np.power(basis_sum,3))

        basisDD_n = np.multiply(basisDD_n, np.matrix(np.power(zd,2)).T) + np.multiply(basisD_n, np.matrix(zdd).T)
        basisD_n = np.multiply(basisD_n, np.matrix(zd).T)

        Basis["Gn"] = basis_n
        Basis["Gndot"] = basisD_n
        Basis["Gnddot"] = basisDD_n

        return Basis

    def leastSquareOnWeights(self,weight,Gn,data):
        weight["demo_q"] = data
        nDemo = weight["nDemo"]
        nJoints = weight["nJoints"]
        nBasis = weight["nBasis"]
        my_linRegRidgeFactor = weight["my_linRegRidgeFactor"]

        MPPI = np.linalg.solve(Gn.T*Gn + my_linRegRidgeFactor, Gn.T)

        w, ind = [], []
        for i in range(nJoints):
            w_j = np.empty((0,nBasis), float)
            for j in range(nDemo):
                w_ = MPPI*data[j][:,i]
                w_j = np.append(w_j,w_.T,axis=0)
            w.append(w_j)
            ind.append(np.matrix(range(i*nBasis,(i+1)*nBasis)))

        weight["index"] = ind
        weight["w_full"] = np.empty((nDemo,0), float)
        for i in range(nJoints):
            weight["w_full"] = np.append(weight["w_full"],w[i],axis=1)

        weight["cov_full"] = np.cov(weight["w_full"].T)
        weight["mean_full"] = np.mean(weight["w_full"],axis=0).T

        return weight

    def Observation(self,stdev,param,p_data,obs_data,observed_data_index=[99]):
        obs = {"joint":param["observedJointPos"], "jointvel":param["observedJointVel"], "stdev":stdev}
        obs["q"] = np.zeros((param["nTotalJoints"],param["nTraj"]))
        obs["qdot"] = np.zeros((param["nTotalJoints"],param["nTraj"]))
        obs["index"] = observed_data_index

        for i in obs["joint"]:
            obs["q"][i,obs["index"]] = obs_data[0,i]

        return obs

    def kfLoop(self,promp,param,obs):
        sigma_obs = obs["stdev"]
        P0 = promp["w"]["cov_full"]
        x0 = promp["w"]["mean_full"]
        R_obs = (sigma_obs**2)*np.identity(2*param["nTotalJoints"])

        for k in obs["index"]:
            H0 = self.observationMatrix(k,promp,obs["joint"],obs["jointvel"])


            z0 = np.empty((0,0), float)
            for i in range(promp["nJoints"]):
                z0 = np.append(z0, obs["q"][i,k])
                z0 = np.append(z0, obs["qdot"][i,k])
            z0 = np.matrix(z0).T

            x0, P0 = self.kfRecursion(x0,P0,H0,z0,R_obs)

            jointKF = self.perJointPromp(x0,P0,promp)

        return jointKF

    def observationMatrix(self,k,p,observedJointPos,observedJointVel):
        nJoints = p["nJoints"]
        nTraj = p["nTraj"]
        Gn = p["basis"]["Gn"]
        Gn_d = p["basis"]["Gndot"]
        normalizedTime = k

        Hq_measured = Gn[normalizedTime,:]
        Hqdot_measured = Gn_d[normalizedTime,:]
        Hq_unmeasured = np.zeros((1, p["nBasis"]))
        Hqdot_unmeasured = np.zeros((1, p["nBasis"]))

        H = []
        for i in range(nJoints):
            if not observedJointPos.all:
                H_temp = Hq_unmeasured
            else:
                if np.sum(i==observedJointPos)==0:
                    H_temp = Hq_unmeasured
                else:
                    H_temp = Hq_measured
            
            if not observedJointVel: #when joint vel not observed
                H_temp = np.append(H_temp, Hqdot_unmeasured, axis = 0)
            else:
                if np.sum(j==observedJointVel)==0:
                    H_temp = np.append(H_temp, Hqdot_unmeasured, axis = 0)
                else:
                    H_temp = np.append(H_temp, Hqdot_measured, axis = 0)

            H.append(H_temp)
        H = linalg.block_diag(*H)

        return H

    def kfRecursion(self,x_old,P_old,H,z,R_obs):
        H, P_old = np.matrix(H), np.matrix(P_old)
        tmp = np.matmul(H,np.matmul(P_old,H.T)) + R_obs
        K = np.matmul(np.matmul(P_old,H.T),np.linalg.inv(tmp))

        P_new = P_old - (K*H*P_old)
        x_new = x_old + K*(z - (H*x_old))

        return x_new, P_new

    def perJointPromp(self,xFull,Pfull,pmp):
        nBasis = pmp["nBasis"]
        nTraj = pmp["nTraj"]

        kf = {"w_mean":[],"w_sigma":[],"w_sigma_ii":[],"q_mean":[],"q_sigma_ii":[],"qdot_mean":[],"qdot_sigma_ii":[]}
        for i in range(pmp["nJoints"]):
            ind = np.array(range(i*nBasis,(i+1)*nBasis))
            kf["w_mean"].append(xFull[ind])
            kf["w_sigma"].append(Pfull[ind,:][:,ind])
            kf["w_sigma_ii"].append(np.diag(kf["w_sigma"][i]))

            q_mean, q_sigma_ii, qdot_mean, qdot_sigma_ii = self.thetaToTraj(kf["w_mean"][i], kf["w_sigma"][i], pmp["basis"], pmp["phase"]["dt"], nTraj)

            kf["q_mean"].append(q_mean)
            kf["q_sigma_ii"].append(q_sigma_ii)
            kf["qdot_mean"].append(qdot_mean)
            kf["qdot_sigma_ii"].append(qdot_sigma_ii)

        return kf

    def thetaToTraj(self,w_mean,P_w,basis,phase_dt,nTraj):
        x_mean = []
        x_sigma_ii = []
        xdot_mean = []
        xdot_sigma_ii = []

        for i in range(nTraj-1):
            timePoint = i*phase_dt
            mu_x,_,sigma_t = self.getDistribtionsAtTimeT1(w_mean,P_w,basis,phase_dt,timePoint)
            x_mean = np.append(x_mean, mu_x[0])
            xdot_mean = np.append(xdot_mean, mu_x[1])
            x_sigma_ii = np.append(x_sigma_ii, sigma_t[0,0])                   #TODO: check indexing
            xdot_sigma_ii = np.append(x_sigma_ii, sigma_t[1,1])

        return x_mean, x_sigma_ii, xdot_mean, xdot_sigma_ii

    def getDistribtionsAtTimeT1(self,w_mu,w_cov,basis,dt,timePoint):
        timePointIndex = int(round(timePoint/dt))

        Psi_t = basis["Gn"][timePointIndex,:]
        Psi_td = basis["Gndot"][timePointIndex,:]
        Psi_tdd = basis["Gnddot"][timePointIndex,:]

        Psi_t1 = basis["Gn"][timePointIndex+1,:]
        Psi_t1d = basis["Gndot"][timePointIndex+1,:]

        Phi_t = np.append(Psi_t.T, Psi_td.T, axis=1)
        Phi_t1 = np.append(Psi_t1.T, Psi_t1d.T, axis=1)
        Phi_td = np.append(Psi_td.T, Psi_tdd.T, axis=1)

        mu_x = Phi_t.T * w_mu
        mu_xd = Phi_td.T * w_mu
        # print w_cov.shape
        sigma_t = Phi_t.T * w_cov * Phi_t
        sigma_t1 = Phi_t1.T * w_cov * Phi_t1
        sigma_t_t1 = Phi_t.T * w_cov * Phi_t1
        sigma_td_half = Phi_td.T * w_cov * Phi_t

        return mu_x, mu_xd, sigma_t #, sigma_t1, sigma_t_t1, sigma_td_half

def main(args):
    rospy.init_node('ProMP', anonymous=True)
    pmp = ProMP()   
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")


if __name__ == '__main__':
    main(sys.argv)