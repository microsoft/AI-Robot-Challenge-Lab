#!/usr/bin/python
import copy
from datetime import time

import intera_interface
import rospy
from geometry_msgs.msg import (
    Pose,
)

from sorting_demo import demo_constants
from real_gripper import PSGGripper

class SawyerRobotControl(object):
    def __init__(self, trajectory_planner, limb="right", hover_distance=0.08, tip_name="right_gripper_tip"):
        """
        :param limb:
        :param hover_distance:
        :param tip_name:
        """
        self.trajectory_planner = trajectory_planner
        self._limb_name = limb  # string
        self._tip_name = tip_name  # string
        self._hover_distance = hover_distance  # in meters
        self._limb = intera_interface.Limb(limb)

        if demo_constants.is_real_robot():
            self._gripper =PSGGripper()
        else:
            self._gripper = intera_interface.Gripper()

        self._robot_enable = intera_interface.RobotEnable()

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    """
    def move_to_start(self, start_angles=None):
        '''
        :param start_angles:
        :return:
        '''
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0] * 7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
    """



    def gripper_open(self):
        """
        :return:
        """
        rospy.logwarn("OPENING GRIPPER")
        self._gripper.open()
        while self._gripper.is_moving() and not rospy.is_shutdown():
            rospy.sleep(0.4)

    def gripper_close(self):
        """
        :return:
        """
        rospy.logwarn("CLOSING GRIPPER")
        self._gripper.close()

        while self._gripper.is_moving() and not rospy.is_shutdown():
            rospy.sleep(0.1)

    def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
        """
        :param joint_angles:
        :param timeout:
        :return:
        """
        if rospy.is_shutdown():
            return
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles, timeout=timeout)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")




    def disable(self):
        """
        
        :return: 
        """
        self._robot_enable.disable()

    def enable(self):
        """
        
        :return: 
        """
        self._robot_enable.enable()
