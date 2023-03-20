#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np
import sys

class Ofsetter:

    def __init__(self, l_pub_topic: str, r_pub_topic: str) -> None:

        rospy.init_node("cmd_vel_controller")
        rospy.Subscriber("/ackermann/right_wheel_angle", Float64, self.RightCallback)
        rospy.Subscriber("/ackermann/left_wheel_angle", Float64, self.LeftCallback)
        self.l_pub = rospy.Publisher(l_pub_topic, Float64, queue_size=1)
        self.r_pub = rospy.Publisher(r_pub_topic, Float64, queue_size=1)
        

    def RightCallback(self, msg: Float64):
        rad_msg = Float64()
        rad_msg.data = msg.data*np.pi/180  #convert to radians
        self.r_pub.publish(rad_msg)

    def LeftCallback(self, msg: Float64):
        rad_msg = Float64()
        rad_msg.data = msg.data*np.pi/180  #convert to radians
        self.l_pub.publish(rad_msg)


if __name__ == "__main__":
    
    if len(sys.argv) < 3:
        print("deg_to_rad converter node needs cml args to be specified")
    else:
        obj = Ofsetter(sys.argv[1], sys.argv[2])
        rospy.spin()


