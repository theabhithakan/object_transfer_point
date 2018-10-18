#!/usr/bin/env python
import roslib
## roslib.load_manifest('handover')
import rospy
import sys, time, os
import numpy as np
from geometry_msgs.msg import Twist
PI = 3.1415926535897
# from handover.msg import skeleton
# from std_msgs.msg import Float64MultiArray


class MoveBase:

    def __init__(self):
    	start = time.time()
        self.move_base = rospy.Publisher("/amp_wpi/twist_command", Twist, queue_size=10)



    def move_right(self):
        # start = time.time()
        # # self.move_base = rospy.Publisher("/amp_wpi/twist_command", Twist, queue_size=10)
        # base = Twist()
        # base.angular.z = -0.2
        # rate = 30
        # r = rospy.Rate(rate)
        # stop = 3.1
        # while time.time()-start < stop:
        #     self.move_base.publish(base)
        #     r.sleep()

        base = Twist()
        angular_speed = -0.2*PI/180
        relative_angle = PI/2
        base.linear.x = 0
        base.linear.y = 0
        base.linear.z = 0
        base.angular.x = 0
        base.angular.y = 0
        base.angular.z = angular_speed

        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < relative_angle):
            self.move_base.publish(base)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        base.angular.z = 0
        move_base.publish




    def move_left(self):
        # start = time.time()
        # # self.move_base = rospy.Publisher("/amp_wpi/twist_command", Twist, queue_size=10)
        # base = Twist()
        # base.angular.z = 0.2
        # rate = 30
        # r = rospy.Rate(rate)
        # stop = 3.1
        # while time.time()-start < stop:
        #     self.move_base.publish(base)
        #     r.sleep()



def main(args): 
    rospy.init_node('MoveBase', anonymous=True)
    numpub = MoveBase()	
    try:
	rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")

if __name__ == '__main__':
    main(sys.argv)