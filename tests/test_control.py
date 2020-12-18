#!/bin/env python3
import time
from copy import deepcopy
from unittest import TestCase

import rospy
import rosunit
from geometry_msgs.msg import PoseStamped

from iiwapy import DOF, ControllerError, Damping, MotionController, Stiffness, control_mode
from iiwapy.motion_controller import MotionPromise


class TestControlModes(TestCase):
    def test_control_mode_switching(self):
        control_mode.position()
        print("Position control mode")
        time.sleep(2)

        control_mode.desired_force(DOF.X, 1, 1000)
        print("Desired force control mode")
        time.sleep(2)

        control_mode.cartesian_impedance(Stiffness(), Damping())
        print("Cartesian impedance control mode")
        self.assertTrue(True)
        time.sleep(2)

        control_mode.position()


class TestMovement(TestCase):
    def test_movement(self):
        control = MotionController()

        promise = control.move_to_joint_position([-0.17, 1.0, 0.14, -1.75, -0.18, -1.19, -0.21])
        self.assertIsInstance(promise, MotionPromise)
        promise.wait_for_result(10)
        self.assertTrue(promise.succeeded)
        time.sleep(1)

        control.move_to_joint_position([0, 0.8078, 0, -1.4905, 0, 0, 0]).wait_for_result(10)
        time.sleep(1)

        pose = control.get_pose()
        self.assertIsInstance(pose, PoseStamped)

        target = deepcopy(pose)
        target.pose.position.x += 0.1

        control.move_to_pose_lin(target, velocity=0.5, acceleration=0.5).wait_for_result(10)
        time.sleep(1)

        target2 = deepcopy(pose)  # move back
        control.move_to_pose_lin(target2, velocity=0.5, acceleration=0.5).wait_for_result(10)

        time.sleep(1)

        # Make invalid move 9 meters forward
        target3 = deepcopy(pose)
        target3.pose.position.x += 9
        promise2 = control.move_to_pose(target3, velocity=0.5, acceleration=0.5)
        self.assertTrue(promise2.is_active())

        with self.assertRaises(ControllerError):
            promise2.wait_for_result(10)

        self.assertFalse(promise2.succeeded)
        self.assertFalse(promise2.active)
        self.assertIsInstance(promise2.error_message, str)

        # None linear motion
        control.move_to_pose(target, velocity=0.5, acceleration=0.5).wait_for_result(10)

        time.sleep(1)

        control.move_to_pose(target2, velocity=0.5, acceleration=0.5).wait_for_result(10)


if __name__ == "__main__":
    rospy.init_node("test_control")
    rosunit.unitrun("iiwapy", "test_movement", TestMovement)
    rosunit.unitrun("iiwapy", "test_control_modes", TestControlModes)
