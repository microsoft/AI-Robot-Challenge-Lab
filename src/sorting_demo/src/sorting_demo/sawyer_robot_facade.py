#!/usr/bin/python
import copy
from datetime import time

import intera_interface
import rospy
from geometry_msgs.msg import (
    Pose,
)

from object_detection import EnvironmentEstimation


class SawyerRobotFacade(object):
    def __init__(self, limb="right", hover_distance=0.15, tip_name="right_gripper_tip"):
        """
        :param limb:
        :param hover_distance:
        :param tip_name:
        """
        self._limb_name = limb  # string
        self._tip_name = tip_name  # string
        self._hover_distance = hover_distance  # in meters
        self._limb = intera_interface.Limb(limb)
        self._gripper = intera_interface.Gripper()

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        """
        :param start_angles:
        :return:
        """
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0] * 7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()

    def pick_loop(self, pose, approach_speed=0.001, approach_time = 3.0, meet_time=2.0, retract_time=2.0):
        """
        Internal state machine for picking
        :param pose:
        :return:
        """
        if rospy.is_shutdown():
            return

        # open the gripper

        self.gripper_open()
        rospy.sleep(0.1)

        # servo above pose
        self._approach(pose, time = approach_time, approach_speed=approach_speed)
        rospy.sleep(1.0)

        # servo to pose
        self._servo_to_pose(pose, time=meet_time)
        #rospy.sleep(1.0)


        if rospy.is_shutdown():
            return

        #rospy.sleep(1.0)
        # close gripper
        self.gripper_close()
        self._gripper.set_object_weight(0.25)

        rospy.sleep(0.1)

        # retract to clear object
        self._retract(time=retract_time)

    def place_loop(self, pose, approach_speed, approach_time, meet_time, retract_time):
        """
        Internal state machine for placing
        :param pose:
        :return:
        """
        if rospy.is_shutdown():
            return
        # servo above pose
        self._approach(pose, time = approach_time, approach_speed=approach_speed)
        rospy.sleep(0.1)

        # servo to pose
        self._servo_to_pose(pose, time=meet_time)
        rospy.sleep(0.1)

        if rospy.is_shutdown():
            return
        # open the gripper
        self.gripper_open()
        rospy.sleep(0.1)

        self.gripper_open()
        self._gripper.set_object_weight(0)

        rospy.sleep(0.1)
        # retract to clear object
        self._retract(time=retract_time)

    def gripper_open(self):
        """
        :return:
        """
        self._gripper.open()
        while self._gripper.is_moving() and not rospy.is_shutdown():
            rospy.sleep(0.1)

    def gripper_close(self):
        """
        :return:
        """
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

    def _approach(self, pose, time, approach_speed=0.001):
        """
        :param pose:
        :param time:
        :param approach_speed:
        :return:
        """
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self._limb.ik_request(approach, self._tip_name)

        #self._limb.set_joint_position_speed(0.0001)
        #self._guarded_move_to_joint_position(joint_angles)
        self._servo_to_pose(approach,time=time)
        rospy.sleep(0.1)
        #self._limb.set_joint_position_speed(0.0001)

    def _retract(self, time=2):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        self._servo_to_pose(ik_pose, time=time)

    def _servo_to_pose(self, target_pose, time=4.0, steps=400.0):
        """ An *incredibly simple* linearly-interpolated Cartesian move """
        r = rospy.Rate(1 / (time / steps))  # Defaults to 100Hz command rate
        current_pose = self._limb.endpoint_pose()
        ik_delta = Pose()
        ik_delta.position.x = (current_pose['position'].x - target_pose.position.x) / steps
        ik_delta.position.y = (current_pose['position'].y - target_pose.position.y) / steps
        ik_delta.position.z = (current_pose['position'].z - target_pose.position.z) / steps
        ik_delta.orientation.x = (current_pose['orientation'].x - target_pose.orientation.x) / steps
        ik_delta.orientation.y = (current_pose['orientation'].y - target_pose.orientation.y) / steps
        ik_delta.orientation.z = (current_pose['orientation'].z - target_pose.orientation.z) / steps
        ik_delta.orientation.w = (current_pose['orientation'].w - target_pose.orientation.w) / steps
        for d in range(int(steps), -1, -1):
            if rospy.is_shutdown():
                return
            ik_step = Pose()
            ik_step.position.x = d * ik_delta.position.x + target_pose.position.x
            ik_step.position.y = d * ik_delta.position.y + target_pose.position.y
            ik_step.position.z = d * ik_delta.position.z + target_pose.position.z
            ik_step.orientation.x = d * ik_delta.orientation.x + target_pose.orientation.x
            ik_step.orientation.y = d * ik_delta.orientation.y + target_pose.orientation.y
            ik_step.orientation.z = d * ik_delta.orientation.z + target_pose.orientation.z
            ik_step.orientation.w = d * ik_delta.orientation.w + target_pose.orientation.w
            joint_angles = self._limb.ik_request(ik_step, self._tip_name)
            if joint_angles:
                self._limb.set_joint_positions(joint_angles)
            else:
                rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

            r.sleep()
        r.sleep()