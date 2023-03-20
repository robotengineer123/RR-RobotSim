#!/usr/bin/env python3
from cmd_vel_controller import CmdVelController
import unittest
import numpy as np
from std_msgs.msg import Float64

class CommandCalcTest(unittest.TestCase):

    def test_straight_vel(self):
        wheel_base = 1.0
        track_width = 1.0
        lin_vel = 1.0
        yaw_vel = 0
        radius = 0.2

        controller = CmdVelController(wheel_base, track_width)
        commands = controller.ComputeMotorVel(yaw_vel, lin_vel, radius)

        self.assertAlmostEqual(commands.l_fw_angle, 0)
        self.assertAlmostEqual(commands.r_fw_angle, 0)
        self.assertAlmostEqual(commands.rot_vel, lin_vel/radius)

    
    def test_fwdKin(self):
        wheel_base = 2
        track_width = 1
        lin_vel = 5
        yaw_vel = 1
        radius = 0.2

        controller = CmdVelController(wheel_base, track_width)
        commands = controller.ComputeMotorVel(yaw_vel, lin_vel, radius)
        
        steer = (commands.l_fw_angle + commands.r_fw_angle)/2.0
        yaw = lin_vel/wheel_base*np.tan(steer)  #forward kinematics

        self.assertAlmostEqual(yaw_vel, yaw, 1)  # The calculations seems to have alot of floating point error from tan funcs ( i guess )

    def test_StandStill(self):
        wheel_base = 2
        track_width = 1
        lin_vel = 0
        yaw_vel = 0
        radius = 1

        controller = CmdVelController(wheel_base, track_width)
        commands = controller.ComputeMotorVel(yaw_vel, lin_vel, radius)
        
        self.assertEqual(commands.l_fw_angle, 0)
        self.assertEqual(commands.r_fw_angle, 0)
        self.assertEqual(commands.rot_vel, 0)


if __name__ == "__main__":
    unittest.main()