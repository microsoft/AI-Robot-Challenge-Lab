#! /usr/bin/env python

from math import fabs

import rospy

import actionlib

from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandFeedback,
    GripperCommandResult,
)

import intera_interface


class GripperActionServer(object):
    def __init__(self):
        self._ns = "/robot/gripper_action_server"
        self._gripper = intera_interface.Gripper()

        # Action Server
        self._server = actionlib.SimpleActionServer(
            self._ns,
            GripperCommandAction,
            execute_cb=self._on_gripper_action,
            auto_start=False)
        self._action_name = rospy.get_name()
        self._server.start()
        self._gripper.set_dead_zone("0.021")

        # Action Feedback/Result
        self.feedback_msg = GripperCommandFeedback()
        self.result_msg = GripperCommandResult()

        # Initialize Parameters
        self._timeout = 5.0


    def _update_feedback(self, position):
        self.feedback_msg.position = self._gripper.get_position()
        self.feedback_msg.effort = self._gripper.get_force()
        self.feedback_msg.stalled = False
        self.feedback_msg.reached_goal = False

        self._server.publish_feedback(self.feedback_msg)

    def _command_gripper_update(self, position,is_close_goal):
        if is_close_goal:
            self._gripper.close()
            rospy.logwarn("cmd: close")
        else:
            self._gripper.open()
            rospy.logwarn("cmd: open")

        #self._gripper.set_position(position)


    def check_success(self, position, close_goal):

        rospy.logwarn("gripping force: "+ str(self._gripper.get_force()))
        rospy.logwarn("gripper position: "+ str(self._gripper.get_position()))
        rospy.logwarn("gripper position deadzone: "+ str(self._gripper.get_dead_zone()))


        if not self._gripper.is_moving():
            success = True
        else:
            success = False

        #success = fabs(self._gripper.get_position() - position) < self._gripper.get_dead_zone()


        rospy.logwarn("gripping success: "+ str(success))

        return success

    def _on_gripper_action(self, goal):
        # Store position and effort from call
        # Position to 0:100 == close:open
        position = goal.command.position
        effort = goal.command.max_effort


        # Check for errors
        if self._gripper.has_error():
            rospy.logerr("%s: Gripper error - please restart action server." %
                         (self._action_name,))
            self._server.set_aborted()


        is_close_goal = position < 0.02

        # Reset feedback/result
        self._update_feedback(position)

        # 20 Hz gripper state rate
        control_rate = rospy.Rate(20.0)

        # Record start time
        start_time = rospy.get_time()

        def now_from_start(start):
            return rospy.get_time() - start

        self._command_gripper_update(position,is_close_goal)
        rospy.sleep(0.2)

        # Continue commanding goal until success or timeout
        while ((now_from_start(start_time) < self._timeout or
               self._timeout < 0.0) and not rospy.is_shutdown()):
            if self._server.is_preempt_requested():
                self._gripper.stop()
                rospy.loginfo("%s: Gripper Action Preempted" %
                              (self._action_name,))
                self._server.set_preempted(self.result_msg)
                return
            self._update_feedback(position)
            if self.check_success(position,is_close_goal):
                self._server.set_succeeded(self.result_msg)
                return
            self._command_gripper_update(position,is_close_goal)
            control_rate.sleep()

        # Gripper failed to achieve goal before timeout/shutdown
        self._gripper.stop()
        if not rospy.is_shutdown():
            rospy.logerr("%s: Gripper Command Not Achieved in Allotted Time" %
                         (self._action_name,))
        self._update_feedback(position)
        self._server.set_aborted(self.result_msg)

import argparse

import rospy

from dynamic_reconfigure.server import Server


def start_server(gripper):
    print("Initializing node... ")
    rospy.init_node("rsdk_gripper_action_server%s" %
                    ("" if gripper == 'both' else "_" + gripper,))
    print("Initializing gripper action server...")

    gas = GripperActionServer()

    print("Running. Ctrl-c to quit")
    rospy.spin()


def main():
    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument("-g", "--gripper", dest="gripper", default="both",
                        choices=['both', 'left', 'right'],
                        help="gripper action server limb",)
    args = parser.parse_args(rospy.myargv()[1:])
    start_server(args.gripper)


if __name__ == "__main__":
    main()