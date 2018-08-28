#!/usr/bin/python
import copy
import math
import numpy
import traceback
from threading import Thread
import utils.mathutils
from  threading import Thread
import concurrent.futures
from concurrent.futures import ThreadPoolExecutor
from threading import Lock
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


def tasync(taskname):
    def wrapper(f):
        @wraps(f)
        def wrapped(self, *f_args, **f_kwargs):

            if self.has_cancel_signal():
                rospy.logerr("trying to invoke but cancel signal: "+ str(taskname))
                self.print_tasks()
                return Task("CANCEL",None)

            if self.pause_flag:
                rospy.logerr("PAUSEEEE")
                while self.pause_flag and not rospy.is_shutdown():
                    rospy.sleep(0.5)
                    rospy.logwarn("Task %s is paused"%taskname)

            tt = Task(taskname, None)

            def lamb():
                res = None
                try:
                    #f_kwargs["task"] = tt
                    res = f(self, *f_args, **f_kwargs)
                except Exception as ex:
                    rospy.logerr("task wrapping error (%s): %s"%(taskname, str(ex)))
                    traceback.print_exc()
                return res

            self.add_task(tt)

            fut = self.executor.submit(lamb)
            tt.future = fut

            def got_result(fut):
                try:
                    rospy.logwarn("removing task: "+ tt.name)
                    self.remove_task(tt)
                except Exception as ex:
                    rospy.logwarn("error at done callback: "  + tt.name + str(ex))

                self.print_tasks()

            fut.add_done_callback(got_result)

            return tt

        return wrapped

    return wrapper


class Task:
    def __init__(self, name, future):
        self.name = name
        self.future = future
        self.marked_cancel = False

    def cancel(self):
        marked_cancel = True
        resultcancel = self.future.cancel()

    def result(self):
        if self.future is not None:
            return self.future.result()
        else:
            return None


class TaskPlanner:
    def __init__(self):
        """
        """
        limb = 'right'
        hover_distance = 0.2  # meters

        # subcomponents
        self.environment_estimation = EnvironmentEstimation()
        self.sawyer_robot = SawyerRobotControl(limb, hover_distance)

        self.tasks = []
        self.executor = ThreadPoolExecutor(max_workers=4)

        self.task_facade = RobotTaskFacade(self)
        self.cancel_signal = False
        self.pause_flag = False

        self.mutex = Lock()

    def has_cancel_signal(self):
        return self.cancel_signal

    def add_task(self, task):
        """
        
        :return: 
        """
        rospy.logwarn("adding task: "+ task.name)
        try:
            self.mutex.acquire()
            self.tasks.append(task)
        finally:
            self.mutex.release()


    def remove_task(self,task):
        """
        
        :param tt: 
        :return: 
        """
        try:
            self.mutex.acquire()
            self.tasks.remove(task)
        finally:
            self.mutex.release()

    def get_task_facade(self):
        """
        
        :return: 
        """
        return self.task_facade

    @tasync("MOVE XY")
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

    @tasync("GO HOME")
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

    @tasync("GREET TASK")
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

    @tasync("GO TO VISION POSE")
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

        self.sawyer_robot._limb.move_to_joint_positions(joint_angles)
        self.delay_task(6).result()

    @tasync("TURN OVER TRAY")
    def create_complete_turn_over_tray(self, target_tray, homepose):
        """
        :return:
        """

        # PICK TRAY
        self.create_pick_task(copy.deepcopy(target_tray.get_tray_pick_location()),
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

        self.create_place_task(copy.deepcopy(target_tray.get_tray_pick_location()),
                               approach_speed=0.0001,
                               approach_time=3.0,
                               meet_time=3.0,
                               retract_time=1.0).result()

    @tasync("TURN OVER TRAY")
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

        # prev = self.sawyer_robot._limb.get_joint_position_speed()
        self.sawyer_robot._limb.set_joint_position_speed(0.0001)
        self.sawyer_robot._limb.move_to_joint_positions(joint_angles, timeout=15.0)

        self.delay_task(1).result()

        joint_angles["right_j5"] -= 3 * math.pi / 4.0

        # prev = self.sawyer_robot._limb.get_joint_position_speed()
        self.sawyer_robot._limb.move_to_joint_positions(joint_angles, timeout=15.0)
        self.delay_task(1).result()

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

    @tasync("APPROACH")
    def create_approach_task(self, target_pose, approach_speed, approach_time, hover_distance=None):
        """
        :param target_pose:
        :param approach_speed:
        :param approach_time:
        :param hover_distance:
        :return:
        """
        self.sawyer_robot._approach(target_pose, approach_time, hover_distance, approach_speed)

    @tasync("PICK")
    def create_pick_task(self, target_pose, approach_speed, approach_time, meet_time, retract_time,
                         hover_distance):
        """
        :param target_pose:
        :param approach_speed:
        :return:
        """
        rospy.logwarn("PICKING")
        return self.sawyer_robot.pick_loop(target_pose, approach_speed, approach_time, meet_time, retract_time,
                                           hover_distance)

    @tasync("PLACE")
    def create_place_task(self, target_pose, approach_speed, approach_time, meet_time, retract_time):
        """
        :param target_pose:
        :return:
        """
        rospy.logwarn("\nPlacing task..." + str(target_pose))
        return self.sawyer_robot.place_loop(target_pose, approach_speed, approach_time, meet_time, retract_time)

    @tasync("SELECT BLOCK&TRAY")
    def create_decision_select_block_and_tray(self, target_block_index):
        """
        :return:
        """
        rospy.logwarn("\nPlacing task...")
        return self.decision_next_block_action(target_block_index)


    def compute_block_pick_offset_transform(self, pose):
        overhead_orientation = Quaternion(
            x=-0.00142460053167,
            y=0.999994209902,
            z=-0.00177030764765,
            w=0.00253311793936)

        overhead_translation = [0,
                                0,
                                -0.25 * demo_constants.CUBE_EDGE_LENGTH]

        pose.orientation = overhead_orientation

        pose.position.x += overhead_translation[0]
        pose.position.y += overhead_translation[1]
        pose.position.z += overhead_translation[2]
        return pose

    def compute_tray_pick_offset_transform(self, pose):
        overhead_orientation = Quaternion(
            x=-0.00142460053167,
            y=0.999994209902,
            z=-0.00177030764765,
            w=0.00253311793936)

        pose.orientation = overhead_orientation
        return pose


    def decision_next_block_action(self, target_block_index):
        """
        :return:
        """

        # An orientation for gripper fingers to be overhead and parallel to the obj

        blocks = self.environment_estimation.get_blocks()

        rospy.logwarn("NEW TARGET BLOCK INDEX: %d" % target_block_index)

        target_block = None
        if blocks is not None and len(blocks) > 0:
            target_block = blocks[target_block_index]  # access first item , pose field

            target_block.final_pose = self.compute_block_pick_offset_transform(target_block.final_pose)
        else:
            rospy.logwarn("OUPS!!")
            return

        target_tray = copy.deepcopy(self.environment_estimation.get_tray_by_color(target_block.get_color()))
        target_tray.final_pose = self.compute_tray_pick_offset_transform(target_tray.final_pose)

        rospy.logwarn("TARGET TRAY POSE: " + str(target_tray))
        return target_block, target_tray

    @tasync("SLEEP")
    def delay_task(self, secs):
        """
        
        :param secs: 
        :return: 
        """
        rospy.sleep(secs)

    @tasync("PICK BLOCK FROM TABLE AND MOVE TO TRAY")
    def pick_block_on_table_and_place_on_tray(self, target_block, target_tray):
        """
        
        :param original_block_poses: 
        :return: 
        """
        rospy.logwarn("target block: " + str(target_block))

        original_block_pose = copy.deepcopy(target_block.final_pose)

        rospy.logwarn("target block pose : " + str(original_block_pose))

        self.create_pick_task(original_block_pose, approach_speed=0.0001, approach_time=2.0,
                              meet_time=3.0,
                              retract_time=1.0,
                              hover_distance=None).result()

        self.create_place_task(copy.deepcopy(target_tray.get_tray_place_block_location()),
                               approach_speed=0.0001,
                               approach_time=2.0,
                               meet_time=3.0,
                               retract_time=1.0).result()

        return original_block_pose

    @tasync("BLOCK FROM TRAY TO TABLE")
    def pick_all_pieces_from_tray_and_put_on_table(self, original_block_pose):
        """
        Pick a block from where it is located on the tray and move it back to the table
        :param original_block_pose: 
        :return: 
        """

        self.environment_estimation.update()
        blocks = self.environment_estimation.get_blocks()
        blocks_count = len(blocks)
        target_block_index = 0

        while target_block_index < blocks_count:
            target_block, target_tray = self.create_detect_block_poses_task(target_block_index).result()

            self.create_pick_task(copy.deepcopy(target_block.final_pose),
                                  approach_speed=0.0001,
                                  approach_time=2.0,
                                  meet_time=3.0,
                                  retract_time=1.0,
                                  hover_distance=None).result()



            place_pose = self.compute_block_pick_offset_transform(original_block_pose[target_block_index])
            #rospy.logerr("place vs: "+ str(target_block.final_pose) +"\n"+ str(place_pose))

            self.create_place_task(copy.deepcopy(place_pose),
                                   approach_speed=0.0001,
                                   approach_time=2.0,
                                   meet_time=3.0,
                                   retract_time=1.0).result()

            target_block_index += 1

    @tasync("OBSERVE ALL CUBES")
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
                self.delay_task(4).result()
                iteration += 1

    @tasync("WAIT FOREVER")
    def create_wait_forever_task(self):
        """
        locks the taskplanner forever
        :return: 
        """
        while True:
            self.delay_task(10).result()

    @tasync("PICK BY COLOR")
    def pick_block_by_color(self, color):
        # stop current robot motion
        # cancel current task

        self.disable_robot_task().result()
        self.enable_robot_task().result()
        #self.create_wait_forever_task().result()


    @tasync("REQUEST PUT ALL CONTENTS ON TABLE")
    def put_all_contents_on_table(self):
        self.environment_estimation.update()
        original_blocks_poses = self.environment_estimation.get_original_block_poses()
        rospy.logwarn(original_blocks_poses)
        self.pick_all_pieces_from_tray_and_put_on_table(original_blocks_poses).result()
        self.create_wait_forever_task().result()


    @tasync("DETECT BLOCK POSE")
    def create_detect_block_poses_task(self, target_block_index):
        target_block = None
        target_tray = None
        while target_block is None:
            rospy.logwarn(" -- ENVIRONMENT ESTIMATION")

            self.environment_estimation.update()
            target_block, target_tray = self.create_decision_select_block_and_tray(target_block_index).result()
            self.delay_task(0.1).result()

        return target_block, target_tray

    @tasync("MOVE ALL CUBES TO TRAY")
    def create_move_all_cubes_to_trays(self):
        """
        Moves all cubes on the table to the trays according with its color
        :return: 
        """
        self.environment_estimation.update()
        blocks = self.environment_estimation.get_blocks()
        blocks_count = len(blocks)

        target_block_index = 0
        original_block_poses = []
        while target_block_index < blocks_count:
            target_block, target_tray = self.create_detect_block_poses_task(target_block_index).result()

            rospy.logwarn(" -- NEW TARGET BLOCK INDEX: %d" % target_block_index)

            # concurrency issue, what if we lock the objectdetection update?

            original_block_pose = self.pick_block_on_table_and_place_on_tray(target_block, target_tray).result()

            original_block_poses.append(original_block_pose)

            # concurrency issue
            self.environment_estimation.get_tray(target_tray.id).notify_contains_block(target_block)
            target_block_index += 1
            rospy.logwarn("target block index: " + str(target_block_index))

        return original_block_poses

    @tasync("DISABLE ROBOT")
    def disable_robot_task(self):
        self.sawyer_robot.disable()

    @tasync("ENABLE ROBOT")
    def disable_robot_task(self):
        self.sawyer_robot.enable()


    @tasync("LOOP SORTING TASK")
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
        # self.create_greet_task().result()

        # self.create_go_vision_head_pose_task().result()

        # self.create_go_home_task().result()

        # self.create_iterate_all_cubes_task(1).result()

        for i in xrange(2):
            self.create_go_home_task().result()

            # self.create_complete_turn_over_tray, target_tray {"homepose": homepose}).result()

            # yield self.create_go_home_task()
            self.reset_cycle()
            # continue

            original_block_poses = self.create_move_all_cubes_to_trays().result()

            self.reset_cycle()

            self.pick_all_pieces_from_tray_and_put_on_table(original_block_poses).result()

            self.delay_task(10).result()

        self.create_wait_forever_task().result()

    def reset_cycle(self):
        for tray in self.environment_estimation.get_trays():
            tray.reset()

    def execute_task(self, fn, args=[]):
        """
        
        :param fn: 
        :return: 
        """
        # INTERRUPT HERE CURRENT TASK AND SAVE STATE


        self.cancel_signal = True

        rospy.logwarn("cancelling all tasks")

        try:
            self.mutex.acquire()
            for task in self.tasks:
                rospy.logwarn("cancelling task: " + task.name)
                task.cancel()
        finally:
            self.mutex.release()

        #wait until all tasks are finished
        while len(self.tasks)>0:
            rospy.sleep(0.2)

        self.print_tasks()

        self.cancel_signal = False

        fn(*args).result()

        self.delay_task(10)

        self.create_main_loop_task().result()

    def pause(self):
        rospy.logwarn("PAUSING TASK PLANNER")
        self.pause_flag = True

    def resume(self):
        rospy.logwarn("RESUMING TASK PLANNER")
        self.pause_flag = False

    def print_tasks(self):
        try:
            self.mutex.acquire()
            tasksstr = "\n".join([str(t.name) for t in self.tasks])
            rospy.logwarn("tasks stack: \n" + tasksstr)
        except Exception as ex:
            rospy.logerr("error printing task stack: " + str(ex))
        finally:
            self.mutex.release()


    def spin(self):
        """
        
        :return: 
        """
        while not rospy.is_shutdown():
            self.environment_estimation.update()
            self.print_tasks()
            rospy.sleep(2.0)
