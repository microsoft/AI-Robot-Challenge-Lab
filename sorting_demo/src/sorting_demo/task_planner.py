#!/usr/bin/python
import copy
import math
import numpy
from threading import Thread
import utils.mathutils
from  threading import Thread
import concurrent.futures
from concurrent.futures import ThreadPoolExecutor

import time
from functools import wraps

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import demo_constants
from robot_control import SawyerRobotControl
from object_detection import EnvironmentEstimation
import moveit_msgs.srv
import moveit_msgs.msg
from functools import wraps

from robot_tasks_facade import RobotTaskFacade

import time
from re import search
from functools import wraps


def task(taskname):
    def wrapper(f):
        @wraps(f)
        def wrapped(self, *f_args, **f_kwargs):
            def lamb():
                res = f(self, *f_args)
                self.tasks.remove((taskname, fut))
                return res

            fut = self.executor.submit(lamb)
            self.tasks.append((taskname, fut))
            return fut

        return wrapped

    return wrapper


class TaskPlanner:
    def __init__(self):
        """
        """
        limb = 'right'
        hover_distance = 0.2  # meters

        # subcomponents
        self.environment_estimation = EnvironmentEstimation()
        self.sawyer_robot = SawyerRobotControl(limb, hover_distance)

        self.target_block = None
        self.target_tray = None
        self.target_block_index = 0

        self.original_block_poses = []
        self.tasks = []
        self.executor = ThreadPoolExecutor(max_workers=4)

        self.task_facade = RobotTaskFacade(self)


    def get_task_facade(self):
        """
        
        :return: 
        """
        return self.task_facade

    @task("MOVE XY")
    def create_move_to_xyz_pr(self, target_pose):
        """
        Uses de 5DOF ik to locate the arm on top of the table with the camera looking ortogonally to the table.
        The free parameter is the yaw (rotation on Z axis)
        :param target_pose: 
        :return: 
        """

        rospy.logwarn("CALLING IK SERVICE")
        ikservice = rospy.ServiceProxy("/sawyer_ik_5d_node/ik", moveit_msgs.srv.GetPositionIK)

        ik_req = moveit_msgs.msg.PositionIKRequest()
        ik_req.robot_state.joint_state.name = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5",
                                               "right_j6"]

        jntangles = self.sawyer_robot._limb.joint_angles()
        ik_req.robot_state.joint_state.position = [jntangles[k] for k in jntangles]
        ik_req.pose_stamped.pose = target_pose
        # ik_req.constraints.ik_link_name = "right_hand_camera_optical"

        rospy.logwarn("CALLING IK SERVICE request: " + str(ik_req))
        resp = ikservice(ik_req)

        rospy.logwarn("SERVICE RESPONSE:" + str(resp))

        targetjoints = dict(zip(resp.solution.joint_state.name, resp.solution.joint_state.position))

        self.sawyer_robot._limb.set_joint_position_speed(0.000001)
        self.sawyer_robot._guarded_move_to_joint_position(targetjoints)

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

    @task("GO HOME")
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

        self.sawyer_robot.move_to_start(starting_joint_angles)


    @task("GREET TASK")
    def create_greet_task(self):
        """
        
        :return: 
        """
        joint_angles_A = {'right_j0': 0.0,
                          'right_j1': 0.0,
                          'right_j2': 0.0,
                          'right_j3': -numpy.pi / 2.0,
                          'right_j4': -numpy.pi / 4.0,
                          'right_j5': 0.0,
                          'right_j6': 0.0}

        joint_angles_B = {'right_j0': 0.0,
                          'right_j1': 0.0,
                          'right_j2': 0.0,
                          'right_j3': -numpy.pi / 2.0,
                          'right_j4': -3 * numpy.pi / 4.0,
                          'right_j5': 0.0,
                          'right_j6': 0.0}

        for i in xrange(4):
            self.sawyer_robot._guarded_move_to_joint_position(joint_angles_A)
            self.sawyer_robot._guarded_move_to_joint_position(joint_angles_B)

    @task("GO TO VISION POSE")
    def create_go_vision_head_pose_task(self):
        """
        :return: 
        """
        # joint_angles = self.sawyer_robot._limb.joint_angles()

        joint_angles = {'right_j0': 0.0,
                        'right_j1': -numpy.pi / 2.0,
                        'right_j2': 0.0,
                        'right_j3': 0.0,
                        'right_j4': 0.0,
                        'right_j5': 0.0,
                        'right_j6': 0.0}

        self.sawyer_robot._limb.move_to_joint_positions (joint_angles)
        rospy.sleep(6)


    @task("TURN OVER TRAY")
    def create_complete_turn_over_tray(self, homepose):
        """
        :return:
        """

        # PICK TRAY
        self.create_pick_task(copy.deepcopy(self.target_tray.get_tray_pick_location()),
                                         approach_speed=0.0001,
                                         approach_time=1.0,
                                         meet_time=0.1,
                                         retract_time=0.1,
                                         hover_distance=0.25).result()

        turnoverpose = copy.deepcopy(homepose)
        turnoverpose.position.x += 0
        turnoverpose.position.y -= 0
        turnoverpose.position.z += 0

        # TARGET POSITION: PRE-TURN OVER
        self.create_approach_task(turnoverpose,
                                             approach_speed=0.0001,
                                             approach_time=3.0,
                                             hover_distance=0.15).result()

        self.create_turn_over_tray_task(turnoverpose).result()

        self.delay_task(3).result()

        self.create_approach_task(turnoverpose,
                                             approach_speed=0.0001,
                                             approach_time=3.0,
                                             hover_distance=0.15).result()

        self.create_place_task(copy.deepcopy(self.target_tray.get_tray_pick_location()),
                                          approach_speed=0.0001,
                                          approach_time=3.0,
                                          meet_time=3.0,
                                          retract_time=1.0).result()

    def create_turn_over_tray_task(self, homepose):
        """
        :param homepose:
        :return:
        """
        # rotate in y

        reverseTransformY = utils.mathutils.rot_y(-1.3 * numpy.pi / 2.0)
        # reverseTransformY = utils.mathutils.rot_x(numpy.pi / 2.0)
        reverseTransformZ = numpy.eye(4)
        # reverseTransformZ = utils.mathutils.rot_z(numpy.pi)

        joint_angles = self.sawyer_robot._limb.joint_angles()
        rospy.logwarn("JOINT ANGLES ON TRAY TURN OVER: " + str(joint_angles))
        joint_angles["right_j6"] -= math.pi

        def movesinglejoint():
            # set better speed

            self.sawyer_robot._limb.move_to_joint_positions(joint_angles, timeout=15.0)

            # set restore speed

        # prev = self.sawyer_robot._limb.get_joint_position_speed()
        self.sawyer_robot._limb.set_joint_position_speed(0.0001)
        self.await(Thread(target=movesinglejoint))
        self.await(self.delay_task(1))

        joint_angles["right_j5"] -= 3 * math.pi / 4.0

        # prev = self.sawyer_robot._limb.get_joint_position_speed()
        self.await(Thread(target=movesinglejoint))
        self.await(self.delay_task(1))

        return self.delay_task(0)

        """
        reverseTransform = utils.mathutils.composition(reverseTransformY,reverseTransformZ)

        homehomopose = utils.mathutils.get_homo_matrix_from_pose_msg(homepose)
        reversedhompose = numpy.matmul(homehomopose, reverseTransform)
        reversedhome = utils.mathutils.homotransform_to_pose_msg(reversedhompose)

        # move tray on table top
        return self.create_approach_task(reversedhome,
                                         approach_speed=0.0001,
                                         approach_time=1.0,
                                         hover_distance=0.1)
        """

    @task("APPROACH")
    def create_approach_task(self, target_pose, approach_speed, approach_time, hover_distance=None):
        """
        :param target_pose:
        :param approach_speed:
        :param approach_time:
        :param hover_distance:
        :return:
        """
        self.sawyer_robot._approach(target_pose, approach_time, hover_distance, approach_speed)


    @task("PICK")
    def create_pick_task(self, target_pose, approach_speed, approach_time, meet_time, retract_time,
                         hover_distance=None):
        """
        :param target_pose:
        :param approach_speed:
        :return:
        """
        return self.sawyer_robot.pick_loop(target_pose, approach_speed, approach_time, meet_time, retract_time, hover_distance)

    @task("PLACE")
    def create_place_task(self, target_pose, approach_speed, approach_time, meet_time, retract_time):
        """
        :param target_pose:
        :return:
        """
        rospy.logwarn("\nPlacing task..." + str(target_pose))
        return self.sawyer_robot.place_loop(target_pose, approach_speed, approach_time, meet_time, retract_time)

    @task("SELECT BLOCK&TRAY")
    def create_decision_select_block_and_tray(self):
        """
        :return:
        """
        rospy.logwarn("\nPlacing task...")
        self.decision_next_block_action()

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

    @task("SLEEP")
    def delay_task(self, secs):
        """
        
        :param secs: 
        :return: 
        """
        rospy.sleep(secs)

    @task("BLOCK FROM TABLE TO TRAY")
    def pick_block_on_table_and_place_on_tray(self, original_block_poses):
        """
        
        :param original_block_poses: 
        :return: 
        """
        original_block_pose = copy.deepcopy(self.target_block.final_pose)
        self.create_pick_task(original_block_pose,
                                         approach_speed=0.0001,
                                         approach_time=2.0,
                                         meet_time=3.0,
                                         retract_time=1.0,
                                         hover_distance=None).result()

        original_block_poses.append(original_block_pose)

        self.create_place_task(copy.deepcopy(self.target_tray.get_tray_place_block_location()),
                                          approach_speed=0.0001,
                                          approach_time=2.0,
                                          meet_time=3.0,
                                          retract_time=1.0).result()

    @task("BLOCK FROM TRAY TO TABLE")
    def pick_all_pieces_from_tray_and_put_on_table(self, original_block_pose):
        """
        Pick a block from where it is located on the tray and move it back to the table
        :param original_block_pose: 
        :return: 
        """

        self.environment_estimation.update()
        blocks = self.environment_estimation.get_blocks()
        blocks_count = len(blocks)

        while self.target_block_index < blocks_count:
            self.create_detect_block_poses_task()

            self.create_pick_task(copy.deepcopy(self.target_block.final_pose),
                                             approach_speed=0.0001,
                                             approach_time=2.0,
                                             meet_time=3.0,
                                             retract_time=1.0,
                                             hover_distance=None).result()

            # yield self.create_go_home_task()

            self.create_place_task(copy.deepcopy(original_block_pose[self.target_block_index]),
                                              approach_speed=0.0001,
                                              approach_time=2.0,
                                              meet_time=3.0,
                                              retract_time=1.0).result()

            self.target_block_index += 1
            self.target_block = None

    @task("OBSERVE ALL CUBES")
    def create_iterate_all_cubes_task(self, iterations_count=None):
        """"
        the robot camera locates on top of each block iteratively and in a loop
        """

        self.environment_estimation.update()
        blocks = self.environment_estimation.get_blocks()
        blocks_count = len(blocks)

        trays_count = len(self.environment_estimation.get_trays())

        iteration = 0
        while iterations_count is None or iteration < iterations_count:
            for block in blocks:
                p = copy.deepcopy(block.pose)
                p.position.z = 0.05

                poseaux = p  # Pose(position=Point(x=0.5 + ki*0.1, y=0.0, z=0.2),orientation=Quaternion(x=0, y=0, z=0, w=1))

                poseauxhomo = utils.mathutils.get_homo_matrix_from_pose_msg(poseaux)
                poseauxhomo = utils.mathutils.composition(poseauxhomo, utils.mathutils.rot_y(math.pi / 2.0))
                poseaux = utils.mathutils.homotransform_to_pose_msg(poseauxhomo)

                self.environment_estimation.update()

                self.create_move_to_xyz_pr(poseaux).result()
                rospy.sleep(4)
                iteration += 1

    @task("WAIT FOREVER")
    def create_wait_forever_task(self):
        """
        locks the taskplanner forever
        :return: 
        """
        while True:
            self.await(self.delay_task(10))

    @task("DETECT BLOCK POSE")
    def create_detect_block_poses_task(self):
        while self.target_block is None:
            rospy.logwarn(" -- ENVIRONMENT ESTIMATION")

            self.environment_estimation.update()
            self.create_decision_select_block_and_tray().result()
            self.delay_task(0.1).result()

    @task("MOVE ALL CUBES TO TRAY")
    def create_move_all_cubes_to_trays(self):
        """
        Moves all cubes on the table to the trays according with its color
        :return: 
        """
        self.environment_estimation.update()
        blocks = self.environment_estimation.get_blocks()
        blocks_count = len(blocks)

        while self.target_block_index < blocks_count:
            self.create_detect_block_poses_task()

            rospy.logwarn(" -- NEW TARGET BLOCK INDEX: %d" % self.target_block_index)

            # concurrency issue, what if we lock the objectdetection update?

            self.pick_block_on_table_and_place_on_tray(self.original_block_poses)

            # concurrency issue
            self.environment_estimation.get_tray(self.target_tray.id).notify_contains_block(self.target_block)
            self.target_block_index += 1
            self.target_block = None

    @task("LOOP SORTING TASK")
    def create_main_loop_task(self):
        """
        This is the main plan of the application
        :return:
        """
        self.create_go_home_task().result()

        """
        home_position = self.sawyer_robot._limb.endpoint_pose()
        pos = home_position["position"]
        q = home_position["orientation"]
        homepose = Pose(position=Point(x=pos.x, y=pos.y, z=pos.z),
                        orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

        rospy.logwarn("home pose:" + str(homepose))
        """


        # for ki in xrange(5):
        self.create_greet_task().result()

        self.create_go_vision_head_pose_task().result()

        self.create_go_home_task().result()

        self.create_iterate_all_cubes_task(1).result()

        for i in xrange(2):
            rospy.logwarn("starting cycle: " + str(self.target_block_index))

            self.create_go_home_task().result()
            # self.target_tray = None

            # self.create_complete_turn_over_tray, {"homepose": homepose}).result()

            # yield self.create_go_home_task()
            self.reset_cycle()
            # continue

            self.create_move_all_cubes_to_trays()

            self.reset_cycle()

            self.pick_all_pieces_from_tray_and_put_on_table(self.original_block_poses)

            self.delay_task(10).result()

        self.create_wait_forever_task()

    def reset_cycle(self):

        self.target_block_index = 0
        self.target_tray = None
        self.target_block = None

        for tray in self.environment_estimation.get_trays():
            tray.reset()

    def run(self):
        """
        main planner loop that executes all the behaviors and open paralelly other tasks with their own loop
        :return: 
        """

        # Move to the desired starting angles

        t = self.create_main_loop_task()

    def execute_task(self, fn):
        """
        
        :param fn: 
        :return: 
        """
        # INTERRUPT HERE CURRENT TASK AND SAVE STATE

        self.await(fn())

    def print_tasks(self):
        try:
            rospy.logwarn("tasks stack: " + str([str(n[0]) + "\n" for n in self.tasks]))
        except Exception as ex:
            rospy.logerr(str(ex))

    def spin(self):
        """
        
        :return: 
        """
        while not rospy.is_shutdown():
            self.environment_estimation.update()
            self.print_tasks()
            rospy.sleep(2.0)
