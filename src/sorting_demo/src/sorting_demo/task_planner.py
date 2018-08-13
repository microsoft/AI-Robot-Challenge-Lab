#!/usr/bin/python
from threading import Thread

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import demo_constants
from sawyer_robot_facade import SawyerRobotFacade


class TaskPlanner:
    def __init__(self):
        limb = 'right'
        hover_distance = 0.15  # meters
        self.sawyer_robot = SawyerRobotFacade(limb, hover_distance)
        self.target_block_pose = None

    def create_go_home_task(self):
        # Starting Joint angles for right arm
        starting_joint_angles = {'right_j0': -0.041662954890248294,
                                 'right_j1': -1.0258291091425074,
                                 'right_j2': 0.0293680414401436,
                                 'right_j3': 2.17518162913313,
                                 'right_j4': -0.06703022873354225,
                                 'right_j5': 0.3968371433926965,
                                 'right_j6': 1.7659649178699421}

        return Thread(target=self.sawyer_robot.move_to_start, args=[starting_joint_angles])

    def create_pick_task(self, target_pose):
        return Thread(target=self.sawyer_robot.pick_loop, args=[target_pose])

    def create_place_task(self, target_pose):
        print("\nPlacing task...")
        return Thread(target=self.sawyer_robot.place_loop, args=[target_pose])

    def create_find_next_block_pose(self):
        print("\nPlacing task...")
        return Thread(target=self.find_next_block_pose)

    def find_next_block_pose(self):
        block_poses = list()

        # An orientation for gripper fingers to be overhead and parallel to the obj
        overhead_orientation = Quaternion(
            x=-0.00142460053167,
            y=0.999994209902,
            z=-0.00177030764765,
            w=0.00253311793936)

        original_pose_block = Pose(
            position=Point(x=0.45, y=0.155, z=-0.129),
            orientation=overhead_orientation)

        overhead_translation = [0.75 * demo_constants.CUBE_EDGE_LENGTH, demo_constants.CUBE_EDGE_LENGTH / 2.0,
                                0.25 * demo_constants.CUBE_EDGE_LENGTH]

        blocks = self.sawyer_robot.environmentEstimation.get_blocks()
        rospy.logwarn("ITERATION!!!")
        rospy.logwarn("blocks: " + str(blocks))

        block_poses.append(original_pose_block)

        if blocks is not None and len(blocks) > 0:
            self.target_block_pose = blocks[0][1]  # access first item , pose field
            self.target_block_pose.orientation = overhead_orientation

            self.target_block_pose.position.x += overhead_translation[0]
            self.target_block_pose.position.y += overhead_translation[1]
            self.target_block_pose.position.z += overhead_translation[2]

            rospy.logwarn(
                "blocks position:" + str(self.sawyer_robot.environmentEstimation.get_blocks()) + "original\n" + str(
                    original_pose_block))

            print("\nPicking task...")

            # return self.target_block
        else:
            rospy.logwarn("OUPS!!")
            # return None

    def async_main_task(self):
        """
        This is the main plan of the application
        :return:
        """
        yield self.create_go_home_task()

        while self.target_block_pose is None:
            yield self.create_find_next_block_pose()
            rospy.sleep(0.5)

        yield self.create_pick_task(self.target_block_pose)
        yield self.create_place_task(self.target_block_pose)

    def create_main_task(self):
        return Thread(target=self.main_task_loop)

    def main_task_loop(self):
        for current_task in self.async_main_task():
            rospy.logwarn("TASK: " + str(self.target_block_pose))

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
            self.sawyer_robot.environmentEstimation.update()
            rospy.sleep(0.1)
