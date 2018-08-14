#!/usr/bin/python
import copy
import math
import numpy
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

    def create_pick_tray_task(self, tray, approach_speed, approach_time, meet_time, retract_time):
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

    def async_create_complete_turn_over_tray(self, homepose):
        """

        :return:
        """
        yield self.create_pick_task(copy.deepcopy(self.target_tray.get_tray_pick_location()),
                                    approach_speed=0.0001,
                                    approach_time=1.0,
                                    meet_time=0.1,
                                    retract_time=0.1,
                                    hover_distance=0.45)

        # move tray on table top
        yield self.create_approach_task(homepose,
                                        approach_speed=0.0001,
                                        approach_time=1.0,
                                        hover_distance=0.15)

        yield self.create_turn_tray_task(homepose)

        yield self.delay_task(3)

        yield self.create_approach_task(homepose,
                                        approach_speed=0.0001,
                                        approach_time=1.0,
                                        hover_distance=0.15)

        yield self.create_place_task(copy.deepcopy(self.target_tray.get_tray_pick_location()),
                                     approach_speed=0.0001,
                                     approach_time=2.0,
                                     meet_time=3.0,
                                     retract_time=1.0)

    def create_turn_tray_task(self, homepose):
        """

        :param homepose:
        :return:
        """
        # rotate in y
        import tf.transformations

        import utils.mathutils
        reverseTransform = utils.mathutils.rot_y(-numpy.pi / 2.0)

        homehomopose = utils.mathutils.get_homo_matrix_from_pose_msg(homepose)
        reversedhompose = numpy.matmul(homehomopose, reverseTransform)
        reversedhome = utils.mathutils.homotransform_to_pose_msg(reversedhompose)

        # move tray on table top
        return self.create_approach_task(reversedhome,
                                         approach_speed=0.0001,
                                         approach_time=1.0,
                                         hover_distance=0.1)

    def create_approach_task(self, target_pose, approach_speed, approach_time, hover_distance=None):
        """

        :param target_pose:
        :param approach_speed:
        :param approach_time:
        :param hover_distance:
        :return:
        """
        return Thread(target=self.sawyer_robot._approach,
                      args=[target_pose, approach_time, hover_distance, approach_speed])

    def create_pick_task(self, target_pose, approach_speed, approach_time, meet_time, retract_time,
                         hover_distance=None):
        """
        :param target_pose:
        :param approach_speed:
        :return:
        """
        return Thread(target=self.sawyer_robot.pick_loop,
                      args=[target_pose, approach_speed, approach_time, meet_time, retract_time, hover_distance])

    def create_place_task(self, target_pose, approach_speed, approach_time, meet_time, retract_time):
        """
        :param target_pose:
        :return:
        """
        rospy.logwarn("\nPlacing task..." + str(target_pose))
        return Thread(target=self.sawyer_robot.place_loop,
                      args=[target_pose, approach_speed, approach_time, meet_time, retract_time])

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

        overhead_translation = [0,
                                0,
                                -0.25 * demo_constants.CUBE_EDGE_LENGTH]

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

        # self.target_tray.final_pose.position.x += overhead_translation[0]
        # self.target_tray.final_pose.position.y += overhead_translation[1]
        # self.target_tray.final_pose.position.z += overhead_translation[2]

        rospy.logwarn("TARGET TRAY POSE: " + str(self.target_tray))

    def delay_task(self, secs):
        return Thread(target=lambda s: rospy.sleep(s), args=[secs])

    def async_main_task(self):
        """
        This is the main plan of the application
        :return:
        """
        yield self.create_go_home_task()
        home_position = self.sawyer_robot._limb.endpoint_pose()
        pos = home_position["position"]
        q = home_position["orientation"]
        homepose = Pose(position=Point(x=pos.x, y=pos.y, z=pos.z),
                        orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

        rospy.logwarn("home pose:" + str(homepose))

        self.environment_estimation.update()
        blocks = self.environment_estimation.get_blocks()
        blocks_count = len(blocks)

        trays_count = len(self.environment_estimation.get_trays())
        original_block_poses = []

        while True:
            rospy.logwarn("starting cycle: "+ str(self.target_block_index))

            while self.target_block_index < blocks_count:
                while self.target_block is None:
                    rospy.logwarn(" -- ENVIRONMENT ESTIMATION")

                    self.environment_estimation.update()
                    yield self.create_decision_select_block_and_tray()
                    yield self.delay_task(0.1)

                rospy.logwarn(" -- NEW TARGET BLOCK INDEX: %d" % self.target_block_index)

                # concurrency issue, what if we lock the objectdetection update?

                # break
                original_block_pose = copy.deepcopy(self.target_block.final_pose)
                yield self.create_pick_task(original_block_pose,
                                            approach_speed=0.0001,
                                            approach_time=2.0,
                                            meet_time=3.0,
                                            retract_time=1.0,
                                            hover_distance=None)

                original_block_poses.append(original_block_pose)

                yield self.create_place_task(copy.deepcopy(self.target_tray.get_tray_place_block_location()),
                                             approach_speed=0.0001,
                                             approach_time=2.0,
                                             meet_time=3.0,
                                             retract_time=1.0)

                # concurrency issue
                self.environment_estimation.get_tray(self.target_tray.id).notify_contains_block(self.target_block)
                self.target_block_index += 1
                self.target_block = None

                # self.target_tray = None

            # yield self.create_go_home_task()
            #self.await_async_task(self.async_create_complete_turn_over_tray, {"homepose" : homepose})


            # yield self.create_go_home_task()
            self.reset_cycle()

            while self.target_block_index < blocks_count:
                while self.target_block is None:
                    rospy.logwarn(" -- ENVIRONMENT ESTIMATION")

                    self.environment_estimation.update()
                    yield self.create_decision_select_block_and_tray()
                    yield self.delay_task(0.1)

                yield self.create_pick_task(copy.deepcopy(self.target_block.final_pose),
                                            approach_speed=0.0001,
                                            approach_time=2.0,
                                            meet_time=3.0,
                                            retract_time=1.0,
                                            hover_distance=None)


                #yield self.create_go_home_task()

                yield self.create_place_task(copy.deepcopy(original_block_poses[self.target_block_index]),
                                             approach_speed=0.0001,
                                             approach_time=2.0,
                                             meet_time=3.0,
                                             retract_time=1.0)

                self.target_block_index += 1
                self.target_block = None

            self.reset_cycle()
            yield self.delay_task(10)


            """
            yield self.create_pick_tray_task(self.environment_estimation.get_trays()[0],
                                             approach_speed=0.0001,
                                             approach_time=1.0,
                                             meet_time=0.1,
                                             retract_time=0.1)
            """

    def reset_cycle(self):

        self.target_block_index = 0
        self.target_tray = None
        self.target_block = None

        for tray in self.environment_estimation.get_trays():
            tray.reset()

    def create_main_task(self):
        """
        :return:
        """
        return Thread(target=self.main_task_loop)

    def main_task_loop(self):
        """
        :return:
        """
        self.await_async_task(self.async_main_task)

    def await_async_task(self, task, args=dict()):
        """
        :param task:
        :return:
        """
        for current_task in task(**args):
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
