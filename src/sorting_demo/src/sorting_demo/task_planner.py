#!/usr/bin/python
import copy
import math
from threading import Thread

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import demo_constants
from sawyer_robot_facade import SawyerRobotFacade
from object_detection import EnvironmentEstimation


class TaskPlanner:
    def __init__(self):
        """
        """
        limb = 'right'
        hover_distance = 0.2  # meters

        # subcomponents
        self.environment_estimation = EnvironmentEstimation()
        self.sawyer_robot = SawyerRobotFacade(limb, hover_distance)

        self.target_block = None
        self.target_tray = None
        self.target_block_index = 0
        self.target_tray_index = 0

    def create_pick_tray_task(self,tray,approach_speed, approach_time, meet_time, retract_time):
        """
        """
        tray = copy.deepcopy(tray)

        p = copy.deepcopy(tray.get_tray_place_block_location())

        rospy.logwarn("PICK TRAY FINAL POSE: " + str(p))

        return self.create_pick_task(p,
                                     approach_speed,
                                     approach_time,
                                     meet_time,
                                     retract_time)

    def create_go_home_task(self):
        """
        :return:
        """
        rospy.loginfo("GO TO HOME TASK")

        # Starting Joint angles for right arm
        starting_joint_angles = {'right_j0': -0.041662954890248294,
                                 'right_j1': -1.0258291091425074,
                                 'right_j2': 0.0293680414401436,
                                 'right_j3': 2.17518162913313,
                                 'right_j4': -0.06703022873354225,
                                 'right_j5': 0.3968371433926965,
                                 'right_j6': 1.7659649178699421}

        return Thread(target=self.sawyer_robot.move_to_start, args=[starting_joint_angles])

    def create_pick_task(self, target_pose, approach_speed, approach_time, meet_time, retract_time):
        """
        :param target_pose:
        :param approach_speed:
        :return:
        """
        return Thread(target=self.sawyer_robot.pick_loop, args=[target_pose, approach_speed, approach_time, meet_time, retract_time])

    def create_place_task(self, target_pose, approach_speed, approach_time, meet_time, retract_time):
        """
        :param target_pose:
        :return:
        """
        rospy.logwarn("\nPlacing task..." + str(target_pose))
        return Thread(target=self.sawyer_robot.place_loop, args=[target_pose, approach_speed, approach_time, meet_time, retract_time])

    def create_decision_select_block_and_tray(self):
        """
        :return:
        """
        rospy.logwarn("\nPlacing task...")
        return Thread(target=self.decision_next_block_action)

    def decision_next_block_action(self):
        """
        :return:
        """

        # An orientation for gripper fingers to be overhead and parallel to the obj
        overhead_orientation = Quaternion(
            x=-0.00142460053167,
            y=0.999994209902,
            z=-0.00177030764765,
            w=0.00253311793936)

        overhead_translation = [0.5 * demo_constants.CUBE_EDGE_LENGTH,
                                0.45 * demo_constants.CUBE_EDGE_LENGTH,
                                0.5 * demo_constants.CUBE_EDGE_LENGTH]

        blocks = self.environment_estimation.get_blocks()

        rospy.logwarn("NEW TARGET BLOCK INDEX: %d" % self.target_block_index)

        if blocks is not None and len(blocks) > 0:
            self.target_block = blocks[self.target_block_index]  # access first item , pose field
            self.target_block.final_pose.orientation = overhead_orientation

            self.target_block.final_pose.position.x += overhead_translation[0]
            self.target_block.final_pose.position.y += overhead_translation[1]
            self.target_block.final_pose.position.z += overhead_translation[2]
        else:
            rospy.logwarn("OUPS!!")
            return

        self.target_tray = copy.deepcopy(self.environment_estimation.get_tray_by_color(self.target_block.get_color()))

        self.target_tray.final_pose.orientation = overhead_orientation

        self.target_tray.final_pose.position.x += overhead_translation[0]
        self.target_tray.final_pose.position.y += overhead_translation[1]
        self.target_tray.final_pose.position.z += overhead_translation[2]

        rospy.logwarn("TARGET TRAY POSE: " + str(self.target_tray))

    def delay_task(self, secs):
        return Thread(target=lambda s: rospy.sleep(s), args=[secs])

    def async_main_task(self):
        """
        This is the main plan of the application
        :return:
        """
        yield self.create_go_home_task()

        blocks_count = len(self.environment_estimation.get_blocks())
        trays_count = len(self.environment_estimation.get_trays())

        while True:
            while self.target_block_index < blocks_count:
                while self.target_block is None:
                    yield self.create_decision_select_block_and_tray()
                    yield self.delay_task(0.1)

                rospy.logwarn(" -- NEW TARGET BLOCK INDEX: %d" % self.target_block_index)
                rospy.logwarn(" -- NEW TARGET TRAY INDEX: %d" % self.target_tray_index)

                # concurrency issue, what if we lock the objectdetection update?

                yield self.create_pick_task(copy.deepcopy(self.target_block.final_pose),
                                            approach_speed=0.0001,
                                            approach_time=2.0,
                                            meet_time=3.0,
                                            retract_time=1.0)

                yield self.create_place_task(copy.deepcopy(self.target_tray.get_tray_place_block_location()),
                                             approach_speed=0.0001,
                                             approach_time = 2.0,
                                             meet_time=3.0,
                                             retract_time=1.0)

                # concurrency issue
                self.environment_estimation.get_tray(self.target_tray.id).notify_contains_block(self.target_block)

                self.target_block_index += 1
                self.target_tray_index = (self.target_tray_index + 1) % trays_count

                self.target_block = None
                #self.target_tray = None

            yield self.create_go_home_task()

            yield self.create_pick_task(copy.deepcopy(self.target_tray.get_tray_place_block_location()),
                                         approach_speed=0.0001,
                                         approach_time=1.0,
                                         meet_time=0.1,
                                         retract_time=0.1)

            """
            yield self.create_pick_tray_task(self.environment_estimation.get_trays()[0],
                                             approach_speed=0.0001,
                                             approach_time=1.0,
                                             meet_time=0.1,
                                             retract_time=0.1)
            """

    def create_main_task(self):
        """
        :return:
        """
        return Thread(target=self.main_task_loop)

    def main_task_loop(self):
        """
        :return:
        """
        for current_task in self.async_main_task():
            current_task.start()
            current_task.join()
            rospy.sleep(0.1)

    def run(self):
        """
        main planner loop that executes all the behaviors and open paralelly other tasks with their own loop
        :return: 
        """

        # Move to the desired starting angles

        t = self.create_main_task()
        t.start()

        while not rospy.is_shutdown():
            self.environment_estimation.update()
            rospy.sleep(2.0)
