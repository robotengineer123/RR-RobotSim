#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np
import sys
from dataclasses import dataclass

@dataclass
class MotorVel:
    l_fw_angle: float = 0
    r_fw_angle: float = 0
    rot_vel: float = 0

class CmdVelController:
    def __init__(self, wheel_base, track_width) -> None:
        self.L = wheel_base
        self.T = track_width
        self.radius = 0.2

    def InitNode(self, l_rope_drive_topic: str, r_rope_drive_topic: str, l_top_wheel_topic, r_top_wheel_topic):
        rospy.init_node("cmd_vel_controller")
        rospy.Subscriber("cmd_vel", Twist, self._ControlCallback)
        rospy.Subscriber("rope_drive/current_drive_radius", Float64, self._RadiusCallback)

        self.r_rd_pub = rospy.Publisher(r_rope_drive_topic, Float64, queue_size=1)
        self.l_rd_pub = rospy.Publisher(l_rope_drive_topic, Float64, queue_size=1)

        self.r_fw_pub = rospy.Publisher(r_top_wheel_topic, Float64, queue_size=1)
        self.l_fw_pub = rospy.Publisher(l_top_wheel_topic, Float64, queue_size=1)

    
    def ComputeMotorVel(self, yaw_vel, lin_vel, radius)->MotorVel:
        
        if (lin_vel == 0):
            return MotorVel()
        
        steer = np.arctan(self.L*abs(yaw_vel)/lin_vel)
        steer_i = 0
        steer_o = 0
        rot_vel = lin_vel/radius
        commands = MotorVel(0, 0, rot_vel)

        if not steer==0:
            r = self.L/np.tan(steer)

            steer_i = np.arctan(self.L/(r - self.T/2.0))
            steer_o = np.arctan(self.L/(r + self.T/2.0))

            if r  >= 0:
                commands = MotorVel(steer_i, steer_o, rot_vel)
            else:
                commands = MotorVel(steer_o, steer_i, rot_vel)
        
        return commands     

    def _ControlCallback(self, msg: Twist):
        yaw_vel = msg.angular.z
        lin_vel = msg.linear.x

        commands = self.ComputeMotorVel(yaw_vel, lin_vel, self.radius)

        self.r_fw_pub.publish(Float64(data=commands.r_fw_angle))
        self.l_fw_pub.publish(Float64(data=commands.l_fw_angle))

        self.r_rd_pub.publish(Float64(data=commands.rot_vel))
        self.l_rd_pub.publish(Float64(data=commands.rot_vel))

    def _RadiusCallback(self, msg: Float64):
       self.radius = msg.data
    

if __name__=="__main__":
    if len(sys.argv) < 7:
        print("This node takes 2 command line arguments: wheel_base, track_width")
    else:
        try:
            wheel_base = float(sys.argv[1])
            track_width = float(sys.argv[2])

            r_rd_topic = str(sys.argv[3])
            l_rd_topic = str(sys.argv[4])
            r_tw_topic = str(sys.argv[5])
            l_tw_topic = str(sys.argv[6])

            controller = CmdVelController(wheel_base, track_width)
            controller.InitNode(l_rd_topic,
                                r_rd_topic,
                                l_tw_topic,
                                r_tw_topic)
            rospy.spin()
            
        except rospy.ROSInterruptException:
            pass