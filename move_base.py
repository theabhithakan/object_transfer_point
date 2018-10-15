#!/usr/bin/env python
import roslib
## roslib.load_manifest('handover')
import rospy
import sys, time, os
import numpy as np
from geometry_msgs.msg import Twist
# from handover.msg import skeleton
# from std_msgs.msg import Float64MultiArray


class NumPub:

    def __init__(self):
    	start = time.time()
        self.num_pub = rospy.Publisher("/amp_wpi/twist_command", Twist, queue_size=10)
        base = Twist()
        base.angular.z = -0.2
        rate = 30
        r = rospy.Rate(rate)
        stop = 3.1
        while time.time()-start < stop:
            self.num_pub.publish(base)
            r.sleep()

def main(args): 
    rospy.init_node('NumPubBase', anonymous=True)
    numpub = NumPub()	
    try:
	rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")

if __name__ == '__main__':
    main(sys.argv)