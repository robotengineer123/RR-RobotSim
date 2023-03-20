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
        offset_msg = Float64()
        offset_msg.data = msg.data + self.r_offset*np.pi/180  #convert to radians
        self.r_pub.publish(offset_msg)

    def LeftCallback(self, msg: Float64):
        offset_msg = Float64()
        offset_msg.data = msg.data*np.pi/180  #convert to radians
        self.l_pub.publish(offset_msg)

if __name__ == "__main__":
    if len(sys.argv) < 5:
        print("offsetter node needs cml args to be specified")
    else:
        obj = Ofsetter(sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]))
        rospy.spin()


