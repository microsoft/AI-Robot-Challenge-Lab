#!/usr/bin/python
import copy
import math
import numpy
from concurrent.futures import ThreadPoolExecutor
from docutils.nodes import target
from threading import Lock

import moveit_msgs.msg
import moveit_msgs.srv
import rospy
from geometry_msgs.msg import Quaternion, Pose

import demo_constants
import utils.mathutils
from environment_estimation import EnvironmentEstimation
from robot_control import SawyerRobotControl
from robot_tasks_facade import RobotTaskFacade
from robot_funcs import force_joint_limits
from tasync import Task, tasync
import tf
import tf.transformations

import time
from re import search
from functools import wraps

from trajectory_planner import TrajectoryPlanner


class TaskPlanner:
    def __init__(self):
        """
        """
        limb = 'right'
        self._hover_distance = 0.1  # meters
        self.place_hover_distance = 0.15

        # subcomponents
        self.environment_estimation = EnvironmentEstimation()

        self.trajectory_planner = TrajectoryPlanner()
        self.sawyer_robot = SawyerRobotControl(self.trajectory_planner, limb, self._hover_distance)

        self.tasks = []
        self.executor = ThreadPoolExecutor(max_workers=8)

        self.ikservice = rospy.ServiceProxy("/sawyer_ik_5d_node/ik", moveit_msgs.srv.GetPositionIK)

        self.cancel_signal = False
        self.pause_flag = False

        self.mutex = Lock()

        self.current_in_hand_block = None
        self.current_in_hand_block_target_tray = None

        self.task_facade = RobotTaskFacade(self)

        self.starting_joint_angles = {'right_j0': -0.041662954890248294,
                                      'right_j1': -1.0258291091425074,
                                      'right_j2': 0.0293680414401436,
                                      'right_j3': 2.17518162913313,
                                      'right_j4': -0.06703022873354225,
                                      'right_j5': 0.3968371433926965,
                                      'right_j6': 0}

        self.joint_names = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"]

    def has_cancel_signal(self):
        """

        :return: 
        """
        return self.cancel_signal

    def add_task(self, task):
        """
        :return: 
        """
        rospy.logwarn("adding task: " + task.name)
        self.print_tasks()
        try:
            self.mutex.acquire()
            rospy.logwarn("adding task mutex: " + task.name)
            self.tasks.append(task)
        finally:
            self.mutex.release()

    def remove_task(self, task):
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

    def robot_sayt2s(self, text):
        """
        :param text: 
        :return: 
        """
        rospy.logwarn("ROBOT SAYS: " + text)

    @tasync("MOVE XY")
    def create_move_XY(self, target_pose):
        """
        Uses de 5DOF ik to locate the arm on top of the table with the camera looking ortogonally to the table.
        The free parameter is the yaw (rotation on Z axis)
        :param target_pose: 
        :return: 
        """

        rospy.logwarn("CALLING IK SERVICE")

        ik_req = moveit_msgs.msg.PositionIKRequest()
        ik_req.robot_state.joint_state.name = self.joint_names

        # instead of using current state seed to find the most suitable pose in the ik, we better use the
        # home position configuration so that it find suitable solutions even if the current state is weird
        # jntangles = self.sawyer_robot._limb.joint_angles()

        jntangles = self.starting_joint_angles
        ik_req.robot_state.joint_state.position = [jntangles[k] for k in jntangles]
        ik_req.pose_stamped.pose = target_pose
        # ik_req.constraints.ik_link_name = "right_hand_camera_optical"

        rospy.logwarn("CALLING IK SERVICE request: " + str(ik_req))
        resp = self.ikservice(ik_req)

        jntspos = list(resp.solution.joint_state.position)
        jntspos = force_joint_limits(jntspos)

        rospy.logwarn("SERVICE RESPONSE:" + str(resp))

        targetjoints = dict(zip(resp.solution.joint_state.name, jntspos))

        rospy.logwarn("Target joints:" + str(targetjoints))
        rospy.logwarn("target jonts: " + str(jntangles))

        # self.sawyer_robot._limb.set_joint_position_speed(0.000001)
        result = False
        while not result:
            result = self.safe_goto_joint_position(targetjoints).result()

        rospy.logwarn("Completed!! current joints: " + str(self.sawyer_robot._limb.joint_angles()))


        # self.sawyer_robot._guarded_move_to_joint_position(targetjoints)

    @tasync("GOTO JOINT GOAL")
    def safe_goto_joint_position(self, targetjoints, attempts=10, hang_on_fail=False):
        """
        
        :param targetjoints: 
        :param attempts: 
        :param hang_on_fail: 
        :return: 
        """

        # self.sawyer_robot._guarded_move_to_joint_position(targetjoints)
        # rospy.logwarn("JOINT VALUES "+  str(targetjoints.values()))
        # rospy.logwarn("JOINT VALUES " + str(targetjoints))

        # rospy.sleep(3)
        # jnts = targetjoints.values()
        # jnts.reverse()

        current_pose = self.sawyer_robot._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        try:
            for i in range(5):
                rospy.logwarn("traj planner move to joint target")
                rospy.logwarn("move joint planner: " + str(targetjoints))
                reached = self.trajectory_planner.move_to_joint_target(targetjoints, currentpose=ik_pose,
                                                                       attempts=i * attempts)

                if not reached:
                    rospy.logwarn("NOT REACHED")
                    if hang_on_fail:
                        return False
                    else:
                        rospy.logwarn("trying with ompl")
                        self.trajectory_planner.group.set_planner_id("OMPL")
                        reached = self.safe_goto_joint_position(targetjoints, attempts=i * attempts,
                                                                hang_on_fail=True).result()
                        self.trajectory_planner.set_default_planner()

                        if reached:
                            rospy.logwarn("PLAN SUCCESS FOUND WITH OMPL")
                            return True

                else:
                    rospy.logwarn("PLAN SUCCESS FOUND")
                    return True

            return False
        except Exception as ex:
            rospy.logerr(ex.message)
            self.create_wait_forever_task().result()

        return False

    def limb_move_to_joint_positions(self, joint_angles, timeout):
        """
        
        :param joint_angles: 
        :param timeout: 
        :return: 
        """

        # self.sawyer_robot._limb.move_to_joint_positions(joint_angles, timeout=15.0)
        self.trajectory_planner.move_to_joint_target(joint_angles)

    def create_pick_tray_task(self, tray, approach_speed, approach_time, meet_time, retract_time):
        """
        
        :param tray: 
        :param approach_speed: 
        :param approach_time: 
        :param meet_time: 
        :param retract_time: 
        :return: 
        """
        tray = copy.deepcopy(tray)

        p = copy.deepcopy(tray.get_tray_place_block_pose())

        rospy.logwarn("PICK TRAY FINAL POSE: " + str(p))

        return self.create_pick_task(p,
                                     approach_speed,
                                     approach_time,
                                     meet_time,
                                     retract_time)

    @tasync("GO HOME")
    def create_go_home_task(self, check_obstacles=True):
        """
        :return:
        """
        rospy.loginfo("GO TO HOME TASK")

        # self.sawyer_robot.move_to_start(starting_joint_angles)
        if not check_obstacles:
            self.sawyer_robot._guarded_move_to_joint_position(self.starting_joint_angles)
        else:
            self.safe_goto_joint_position(self.starting_joint_angles).result()

    @tasync("GREET TASK")
    def create_greet_task(self):
        """
        :return: 
        """
        self.create_go_home_task().result()
        joint_angles_A = {'right_j0': 0.0,
                          'right_j1': 0.4,
                          'right_j2': 0.0,
                          'right_j3': -numpy.pi / 2.0 - 0.4,
                          'right_j4': -numpy.pi / 4.0,
                          'right_j5': 0.0,
                          'right_j6': 0.0}

        joint_angles_B = {'right_j0': 0.0,
                          'right_j1': 0.4,
                          'right_j2': 0.0,
                          'right_j3': -numpy.pi / 2.0 - 0.4,
                          'right_j4': -3 * numpy.pi / 4.0,
                          'right_j5': 0.0,
                          'right_j6': 0.0}

        for i in xrange(4):
            self.safe_goto_joint_position(joint_angles_A).result()
            self.safe_goto_joint_position(joint_angles_B).result()

    @tasync("GO TO VISION POSE")
    def create_go_vision_head_pose_task(self):
        """
        :return: 
        """
        self.trajectory_planner.ceilheight = 0.95
        self.trajectory_planner.update_ceiling_obstacle()

        # joint_angles = self.sawyer_robot._limb.joint_angles()

        oldceil = self.trajectory_planner.ceilheight

        joint_angles = {'right_j0': 0.0,
                        'right_j1': -numpy.pi / 2.0,
                        'right_j2': 0.0,
                        'right_j3': numpy.pi / 2.0,
                        'right_j4': 0.0,
                        'right_j5': 0.0,
                        'right_j6': 0.0}

        self.trajectory_planner.ceilheight = 2.0

        # self.sawyer_robot._limb.move_to_joint_positions(joint_angles)
        found = self.safe_goto_joint_position(joint_angles).result()
        # self.delay_task(0.2).result()
        rospy.sleep(0.5)
        self.trajectory_planner.ceilheight = oldceil
        """
        rospy.logwarn(self.sawyer_robot._limb.endpoint_pose())

        targetpose = Pose()
        targetpose.position.x = 0.4595219280890743
        targetpose.position.y = 0.1473752184292072
        targetpose.position.z = 0.019578584407653032

        targetpose.orientation.x = -0.011325648436031916
        targetpose.orientation.y = 0.9998115142702567
        targetpose.orientation.z = -0.006101035043221461
        targetpose.orientation.w = 0.014541079448283218

        found = self.trajectory_planner.move_to_cartesian_target(targetpose)
        """

        if not found:
            self.create_wait_forever_task().result()

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
        self.limb_move_to_joint_positions(joint_angles, timeout=15.0)
        # self.sawyer_robot._limb.move_to_joint_positions(joint_angles, timeout=15.0)

        self.delay_task(1).result()

        joint_angles["right_j5"] -= 3 * math.pi / 4.0

        # prev = self.sawyer_robot._limb.get_joint_position_speed()
        # self.sawyer_robot._limb.move_to_joint_positions(joint_angles, timeout=15.0)
        self.limb_move_to_joint_positions(joint_angles, timeout=15.0)

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
        self.approach_hover(target_pose, approach_time, hover_distance, approach_speed).result()

    @tasync("PICK")
    def create_pick_task(self, target_pose, approach_speed, approach_time, meet_time, retract_time,
                         hover_distance):
        """
        :param target_pose:
        :param approach_speed:
        :return:
        """
        rospy.logwarn("PICKING")
        return self.create_pick_loop_task(target_pose, approach_speed, approach_time, meet_time, retract_time,
                                          hover_distance).result()

    @tasync("RETRACT LOOP")
    def _retract_loop(self, time=2, hover_distance=None):
        """

        :param time: 
        :param hover_distance: 
        :return: 
        """
        if hover_distance is None:
            hover_distance = self._hover_distance

        # retrieve current pose from endpoint
        current_pose = self.sawyer_robot._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        self.create_linear_motion_task(ik_pose, time=time).result()

    @tasync("PICK LOOP")
    def create_pick_loop_task(self, target_pose, approach_speed=0.001, approach_time=3.0, meet_time=2.0,
                              retract_time=2.0,
                              hover_distance=None):
        """
        Internal state machine for picking
        :param target_pose:
        :return:
        """
        self.sawyer_robot._limb.set_joint_position_speed(1.0)

        if hover_distance is None:
            hover_distance = self._hover_distance

        final_joints = self.sawyer_robot._limb.ik_request(target_pose, self.sawyer_robot._tip_name,
                                                          joint_seed=self.starting_joint_angles,
                                                          nullspace_goal=self.starting_joint_angles)
        hover_pose = copy.deepcopy(target_pose)
        hover_pose.position.z += hover_distance
        # rospy.logwarn("SEED pick: "+ str(jntsseed))
        rospy.logwarn(final_joints)
        approach_joints = self.sawyer_robot._limb.ik_request(hover_pose, self.sawyer_robot._tip_name,
                                                             joint_seed=final_joints, nullspace_goal=final_joints)

        if rospy.is_shutdown():
            return

        # open the gripper

        self.sawyer_robot.gripper_open()
        rospy.sleep(0.1)

        rospy.logwarn("APPROACHING, hover: " + str(hover_distance))

        # servo above pose
        # success = self.approach_hover(target_pose, time=approach_time, approach_speed=approach_speed, hover_distance=hover_distance).result()
        self.safe_goto_joint_position(approach_joints).result()
        rospy.sleep(0.1)

        rospy.logwarn("APPROACHING DOWN")

        self.trajectory_planner.table1_z = -1.0
        self.trajectory_planner.enable_orientation_constraint = True
        self.create_linear_motion_task(target_pose, time=1.0).result()
        # self.safe_goto_joint_position(final_joints).result()

        rospy.sleep(0.1)

        if rospy.is_shutdown():
            return

        # rospy.sleep(1.0)
        # close gripper
        self.sawyer_robot.gripper_close()
        self.sawyer_robot._gripper.set_object_weight(0.25)

        rospy.sleep(0.1)

        self.safe_goto_joint_position(approach_joints).result()
        self.create_linear_motion_task(hover_pose, time=1.0).result()
        # self.trajectory_planner.enable_orientation_constraint = False
        self.trajectory_planner.enable_orientation_constraint = False

        self.trajectory_planner.set_default_tables_z()
        # retract to clear object
        # self._retract_loop(time=retract_time, hover_distance=hover_distance).result()

    @tasync("LINEAR MOTION")
    def create_linear_motion_task_planner(self, target_pose, time=4.0, steps=400.0):
        """ An *incredibly simple* linearly-interpolated Cartesian move """

        # self.trajectory_planner.enable_collision_table1 = False
        # self.trajectory_planner.clear_parameters()
        # self.trajectory_planner.scene.remove_world_object("table1")
        # rospy.sleep(1.0)

        self.trajectory_planner.table1_z = -1.0
        rospy.logwarn("Targetpose:" + str(target_pose))
        jnts = self.sawyer_robot._limb.ik_request(target_pose, self.sawyer_robot._tip_name)
        rospy.logwarn("JNTS:" + str(jnts))
        success = self.safe_goto_joint_position(jnts).result()
        # success = self.trajectory_planner.move_to_joint_target(jnts.values(),attempts=300)
        self.trajectory_planner.table1_z = 0.0

        if not success:
            self.create_wait_forever_task().result()

        return True

    @tasync("LINEAR MOTION")
    def create_linear_motion_task(self, target_pose, time=4.0, steps=500):
        """ An *incredibly simple* linearly-interpolated Cartesian move """

        r = rospy.Rate(1 / (time / steps))  # Defaults to 100Hz command rate
        current_pose = self.sawyer_robot._limb.endpoint_pose()
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
            joint_angles = self.sawyer_robot._limb.ik_request(ik_step, self.sawyer_robot._tip_name)
            if joint_angles:
                self.sawyer_robot._limb.set_joint_positions(joint_angles)
            else:
                rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

            r.sleep()
        r.sleep()

    @tasync("APPROACH HOVER")
    def approach_hover(self, pose, time, hover_distance, approach_speed=0.001):
        """
        :param pose:
        :param time:
        :param approach_speed:
        :return:
        """
        approach_pose = copy.deepcopy(pose)
        rospy.logwarn("approach pose:" + str(approach_pose))
        rospy.logwarn("hover distance:" + str(hover_distance))
        # approach with a pose the hover-distance above the requested pose

        rospy.logwarn("approach prev z :" + str(approach_pose.position.z))
        approach_pose.position.z = approach_pose.position.z + hover_distance
        rospy.logwarn("approach pos z :" + str(approach_pose.position.z))
        # joint_angles = self._limb.ik_request(approach, self._tip_name)

        # self._limb.set_joint_position_speed(0.0001)
        # self._guarded_move_to_joint_position(joint_angles)
        success = self.create_linear_motion_task(approach_pose, time=time).result()

        if not success:
            self.create_wait_forever_task().result()

        rospy.sleep(0.1)
        # self._limb.set_joint_position_speed(0.0001)

    @tasync("PLACE")
    def create_place_task(self, target_pose, approach_speed, approach_time, meet_time, retract_time):
        """
        :param target_pose:
        :return:
        """
        rospy.logwarn("\nPlacing task..." + str(target_pose))

        hover_distance = self.place_hover_distance
        self.sawyer_robot._limb.set_joint_position_speed(1.0)

        # final_joints = self.sawyer_robot._limb.ik_request(target_pose, self.sawyer_robot._tip_name)

        final_joints = self.sawyer_robot._limb.ik_request(target_pose, self.sawyer_robot._tip_name,
                                                          joint_seed=self.starting_joint_angles,
                                                          nullspace_goal=self.starting_joint_angles)

        hover_pose = copy.deepcopy(target_pose)
        hover_pose.position.z += hover_distance
        # rospy.logwarn("SEED pick: "+ str(jntsseed))
        approach_joints = self.sawyer_robot._limb.ik_request(hover_pose, self.sawyer_robot._tip_name,
                                                             joint_seed=final_joints)

        if rospy.is_shutdown():
            return
            # servo above pose
        # self.approach_hover(target_pose, time=approach_time, approach_speed=approach_speed,
        #                    hover_distance=self._hover_distance).result()

        self.safe_goto_joint_position(approach_joints).result()

        rospy.sleep(0.1)

        rospy.logwarn("APPROACHING DOWN")

        self.trajectory_planner.table2_z = -1.0
        self.trajectory_planner.enable_orientation_constraint = True
        self.create_linear_motion_task(target_pose, time=1.0).result()
        # self.safe_goto_joint_position(final_joints).result()

        rospy.sleep(0.5)

        # self.create_linear_motion_task(target_pose, time=meet_time).result()
        rospy.sleep(0.1)

        if rospy.is_shutdown():
            return
        # open the gripper
        self.sawyer_robot.gripper_open()
        rospy.sleep(0.1)

        self.sawyer_robot._gripper.set_object_weight(0)

        rospy.sleep(0.1)

        rospy.logwarn("RETRACTING")
        # retract to clear object
        # self.safe_goto_joint_position(approach_joints).result()
        self.create_linear_motion_task(hover_pose, time=1.0).result()
        self.trajectory_planner.enable_orientation_constraint = False
        self.trajectory_planner.set_default_tables_z()

    @tasync("SELECT BLOCK&TRAY")
    def create_decision_select_block_and_tray(self, blocks, target_block_index):
        """
        :return:
        """
        # An orientation for gripper fingers to be overhead and parallel to the obj
        rospy.logwarn("NEW TARGET BLOCK INDEX: %d" % target_block_index)

        target_block = None
        if blocks is not None and len(blocks) > 0:
            target_block = blocks[target_block_index]  # access first item , pose field
        else:
            rospy.logwarn("No block to pick from table!!")
            return False

        target_tray = self.environment_estimation.get_tray_by_color(target_block.get_color())
        target_tray.gazebo_pose = self.compute_tray_pick_offset_transform(target_tray.gazebo_pose)

        rospy.logwarn("TARGET TRAY POSE: " + str(target_tray))
        return target_block, target_tray

    def compute_grasp_pose_offset(self, pose):
        """
        :param pose: 
        :return: 
        """

        yrot = tf.transformations.quaternion_from_euler(0, math.pi, 0)

        cubeorientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        # oorient = [overhead_orientation.x,overhead_orientation.y,overhead_orientation.z,overhead_orientation.w]

        # resultingorient = tf.transformations.quaternion_multiply(cubeorientation, tf.transformations.quaternion_conjugate(oorient))
        resultingorient = tf.transformations.quaternion_multiply(cubeorientation, yrot)

        # resultingorient = cubeorientation


        pose.orientation = Quaternion(x=resultingorient[0], y=resultingorient[1], z=resultingorient[2],
                                      w=resultingorient[3])

        pose.position.x += 0
        pose.position.y += 0
        pose.position.z = demo_constants.TABLE_HEIGHT + demo_constants.CUBE_EDGE_LENGTH
        return pose

    def compute_tray_pick_offset_transform(self, pose):
        """
        :param pose: 
        :return: 
        """
        overhead_orientation = Quaternion(
            x=-0.00142460053167,
            y=0.999994209902,
            z=-0.00177030764765,
            w=0.00253311793936)

        pose.orientation = overhead_orientation
        return pose

    @tasync("SLEEP")
    def delay_task(self, secs):
        """
        :param secs: 
        :return: 
        """
        if not rospy.is_shutdown():
            rospy.sleep(secs)

    @tasync("MOVEIT TRAY PLACE")
    def moveit_tray_place(self, target_block, target_tray):
        result = False
        while not result or result < 0:
            self.trajectory_planner.set_default_tables_z()
            self.trajectory_planner.table2_z = demo_constants.TABLE_HEIGHT - 0.05
            self.trajectory_planner.update_table2_collision()
            self.trajectory_planner.update_table1_collision()
            target_block.tray = target_tray
            target_block.tray_place_pose = self.compute_grasp_pose_offset(target_tray.get_tray_place_block_pose())
            result = self.trajectory_planner.place(target_block)
            rospy.logwarn("place result: " + str(result))

    @tasync("MOVEIT TABLETOP PICK")
    def moveit_tabletop_pick(self, target_block):
        # self.sawyer_robot.gripper_open()
        result = False
        while not result or result < 0:
            rospy.logwarn("target block: " + str(target_block))

            target_block.grasp_pose = copy.deepcopy(
                self.compute_grasp_pose_offset(target_block.tabletop_arm_view_estimated_pose))
            rospy.logwarn("target block pose : " + str(target_block.grasp_pose))
            self.trajectory_planner.set_default_tables_z()
            self.trajectory_planner.table1_z = demo_constants.TABLE_HEIGHT - 0.05
            self.trajectory_planner.update_table1_collision()
            result = self.trajectory_planner.pick(target_block,"table1")
            rospy.logwarn("pick result: " + str(result))


    @tasync("MOVEIT TABLETOP PLACE")
    def moveit_tabletop_place(self,target_block):
        result = False
        while not result or result < 0:
            self.trajectory_planner.set_default_tables_z()
            self.trajectory_planner.table1_z = demo_constants.TABLE_HEIGHT - 0.05
            self.trajectory_planner.update_table2_collision()
            self.trajectory_planner.update_table1_collision()
            target_block.tray_place_pose = self.compute_grasp_pose_offset(target_block.tabletop_arm_view_estimated_pose)

            result = self.trajectory_planner.place(target_block)
            rospy.logwarn("place result: " + str(result))

    @tasync("MOVEIT TRAYTOP PICK")
    def moveit_traytop_pick(self, target_block):
        # self.sawyer_robot.gripper_open()
        result = False
        while not result or result < 0:
            rospy.logwarn("target block: " + str(target_block))

            target_block.grasp_pose = self.compute_grasp_pose_offset(target_block.traytop_arm_view_estimated_pose)

            rospy.logwarn("target block pose : " + str(target_block.grasp_pose))
            self.trajectory_planner.set_default_tables_z()
            self.trajectory_planner.table2_z = demo_constants.TABLE_HEIGHT - 0.05
            self.trajectory_planner.update_table2_collision()

            result = self.trajectory_planner.pick(target_block,"table2")
            rospy.logwarn("pick result: " + str(result))


    @tasync("PICK BLOCK FROM TABLE AND MOVE TO TRAY")
    def pick_block_on_table_and_place_on_tray(self, target_block, target_tray):
        """
        :param original_block_poses: 
        :return: 
        """

        # original_block_pose = copy.deepcopy(target_block.gazebo_pose)

        # original_block_pose = copy.deepcopy(target_block.arm_view_estimated_pose)


        try:
            self.trajectory_planner.ceilheight = 0.7

            self.trajectory_planner.register_box(target_block)

            self.moveit_tabletop_pick(target_block).result()

            rospy.sleep(0.5)

            self.moveit_tray_place(target_block, target_tray).result()

            target_tray.notify_contains_block(target_block)
            self.environment_estimation.table.notify_block_removed(target_block)

            rospy.logwarn("pick and place finished. table blocks: "+ str(self.environment_estimation.table.blocks))
            rospy.logwarn("pick and place finished. target tray blocks: "+ str(target_tray.notify_contains_block))

        except:
            self.create_wait_forever_task().result()

        # self.create_wait_forever_task().result()

        # self.trajectory_planner.ceilheight = 0.8
        # self.create_pick_task(grasp_block_pose, approach_speed=0.01, approach_time=1.0,
        #                      meet_time=1.0,
        #                      retract_time=0.5,
        #                      hover_distance=None).result()



        # self.create_place_task(
        #    copy.deepcopy(self.compute_block_pick_offset_transform(target_tray.get_tray_place_block_pose())),
        #    approach_speed=0.0001,
        #    approach_time=2.0,
        #    meet_time=3.0,
        #    retract_time=1.0).result()

        self.trajectory_planner.ceilheight = 0.75
        self.trajectory_planner.set_default_tables_z()

        return target_block.grasp_pose

    @tasync("BLOCK FROM TRAY TO TABLE")
    def pick_all_pieces_from_tray_and_put_on_table(self):
        """
        Pick a block from where it is located on the tray and move it back to the table
        :param original_block_pose: 
        :return: 
        """

        self.environment_estimation.update()

        for target_tray in self.environment_estimation.get_trays():
            for target_block in target_tray.blocks:

                if not demo_constants.SIMULATE_TRAY_BLOCK_DETECTION:
                    detected = self.create_move_top_block_view_and_detect(target_block, "tray_place_pose", additional_z_offset=0.1, CUBE_SIZE=95).result()
                else:
                    rospy.sleep(1.0)
                    self.environment_estimation.update()
                    target_block.traytop_arm_view_estimated_pose = target_block.gazebo_pose
                    target_block.traytop_arm_view_estimated_pose.orientation.x = 0
                    target_block.traytop_arm_view_estimated_pose.orientation.y = 0
                    target_block.traytop_arm_view_estimated_pose.orientation.z = 0
                    target_block.traytop_arm_view_estimated_pose.orientation.w = 1.0

                self.moveit_traytop_pick(target_block).result()
                self.moveit_tabletop_place(target_block).result()

                target_block.tray.blocks.remove(target_block)
                target_block.tray = None

                # target_block, target_tray = self.create_detect_block_poses_task(blocks, target_block_index) \
                #    .result()

                # self.create_pick_task(copy.deepcopy(target_block.gazebo_pose),
                #                      approach_speed=0.0001,
                #                      approach_time=2.0,
                #                      meet_time=3.0,
                #                      retract_time=1.0,
                #                      hover_distance=None).result()

                # place_pose = self.compute_grasp_pose_offset(target_block.grasp_pose)
                #place_pose = target_block.grasp_pose
                # rospy.logerr("place vs: "+ str(target_block.gazebo_pose) +"\n"+ str(place_pose))

                #self.create_place_task(copy.deepcopy(place_pose),
                #                       approach_speed=0.0001,
                #                       approach_time=2.0,
                #                       meet_time=3.0,
                #                       retract_time=1.0).result()

                # target_block_index += 1

    @tasync("HEAD VISION PROCESSING")
    def create_head_vision_processing_on_table(self):
        """
        :return: 
        """
        self.environment_estimation.update()
        self.environment_estimation.compute_block_pose_estimations_from_head_camera()

        return self.environment_estimation.table.blocks

    @tasync("LOCATE ARMVIEW TO BLOCK ESTIMATION")
    def create_move_top_block_view_and_detect(self, block, source="headview_pose_estimation", additional_z_offset=0.0, CUBE_SIZE=150):
        """
        :return: 
        """
        tabletop = True
        if source == "headview_pose_estimation":
            tabletop=True
        elif source == "tray_place_pose":
            tabletop=False

        if tabletop:
            rospy.loginfo("trying to estimate pose of block: " + str(block))
            top_view_pose = copy.deepcopy(block.headview_pose_estimation)
        else:
            rospy.loginfo("trying to estimate pose of block: " + str(block))
            top_view_pose = copy.deepcopy(block.tray_place_pose)
            top_view_pose.orientation.x = 0
            top_view_pose.orientation.y = 0
            top_view_pose.orientation.z = 0
            top_view_pose.orientation.w = 1

        # chose z plane
        top_view_pose.position.z = demo_constants.ARM_TOP_VIEW_Z_OFFSET + additional_z_offset

        poseaux = top_view_pose  # Pose(position=Point(x=0.5 + ki*0.1, y=0.0, z=0.2),orientation=Quaternion(x=0, y=0, z=0, w=1))
        topview_homo_pose = utils.mathutils.get_homo_matrix_from_pose_msg(poseaux)
        topview_homo_pose = utils.mathutils.composition(topview_homo_pose, utils.mathutils.rot_y(math.pi / 2.0))
        poseaux = utils.mathutils.homotransform_to_pose_msg(topview_homo_pose)

        self.create_move_XY(poseaux).result()
        # individual processing algorithm

        estimated_cube_pose = self.environment_estimation.compute_block_pose_estimation_from_arm_camera(CUBE_SIZE=CUBE_SIZE)

        if estimated_cube_pose is None:
            rospy.logerr("cube on table not detected")
            return False

        self.environment_estimation.update()

        if estimated_cube_pose is None:
            rospy.logwarn("cube not detected")
            self.create_wait_forever_task().result()
        else:
            rospy.logwarn("CUBE POSE DETECTED")

        if tabletop:
            block.tabletop_arm_view_estimated_pose = estimated_cube_pose
        else:
            block.traytop_arm_view_estimated_pose = estimated_cube_pose

        return True

    @tasync("OBSERVE ALL CUBES")
    def create_visit_all_cubes_armview(self, iterations_count=None):
        """"
        the robot camera locates on top of each block iteratively and in a loop
        """
        blocks = self.create_head_vision_processing_on_table().result()

        iteration = 0

        while iterations_count is None or iteration < iterations_count:
            for block in blocks:
                detected = self.create_move_top_block_view_and_detect(block).result()

            iteration += 1

    @tasync("WAIT FOREVER")
    def create_wait_forever_task(self):
        """
        locks the taskplanner forever
        :return: 
        """
        while not rospy.is_shutdown():
            self.delay_task(10).result()

    @tasync("PICK BY COLOR")
    def pick_block_on_table_by_color(self, color):
        """
        :param color: 
        :return: 
        """
        blocks = self.environment_estimation.table.get_blocks()
        btarget = [i for i, b in enumerate(blocks) if b.is_color(color)][0]

        target_block, target_tray = self.create_detect_block_poses_task(blocks, btarget).result()

        self.create_pick_task(target_block.gazebo_pose, approach_speed=0.0001, approach_time=2.0,
                              meet_time=3.0,
                              retract_time=1.0,
                              hover_distance=None).result()

        self.current_in_hand_block = target_block
        self.current_in_hand_block_target_tray = target_tray

        # self.create_pick_task(btarget.gazebo_pose)
        return target_block

    @tasync("PICK BY COLOR AND PUT TRAY")
    def put_block_into_tray_task(self, color, trayid):
        """
        :param color: 
        :param trayid: 
        :return: 
        """
        # self.reset_cycle()
        # decide and select block and pick it
        target_block = self.pick_block_on_table_by_color(color).result()

        rospy.logwarn("put block into tray: " + str(self.environment_estimation.get_trays()))
        target_tray = self.environment_estimation.get_tray_by_num(trayid)
        rospy.logwarn("put block into tray: " + str(target_tray))

        place_location = self.compute_grasp_pose_offset(
            copy.deepcopy(target_tray.get_tray_place_block_pose()))

        self.create_place_task(place_location,
                               approach_speed=0.0001,
                               approach_time=3.0,
                               meet_time=3.0,
                               retract_time=1.0).result()

        self.environment_estimation.table.notify_block_removed(target_block)
        target_tray.notify_contains_block(target_block)

    @tasync("REQUEST PUT ALL CONTENTS ON TABLE")
    def put_all_contents_on_table(self):
        """
        :return: 
        """
        self.environment_estimation.update()
        original_blocks_poses = self.environment_estimation.get_original_block_poses()
        rospy.logwarn(original_blocks_poses)
        self.pick_all_pieces_from_tray_and_put_on_table().result()
        self.create_wait_forever_task().result()

    @tasync("DETECT BLOCK POSE")
    def create_detect_block_poses_task(self, blocks, target_block_index):
        """
        :param target_block_index: 
        :return: 
        """
        target_block = None
        target_tray = None
        while target_block is None:
            target_block, target_tray = self.create_decision_select_block_and_tray(blocks, target_block_index).result()
            self.delay_task(0.1).result()

        return target_block, target_tray

    @tasync("MOVE ALL CUBES TO TRAY")
    def create_move_all_cubes_to_trays(self, blocks):
        """
        Moves all cubes on the table to the trays according with its color
        :return: 
        """
        blocks_count = len(blocks)

        while blocks_count > 0:
            # self.create_go_home_task().result()

            target_block, target_tray = self.create_detect_block_poses_task(blocks, 0).result()

            self.trajectory_planner.ceilheight = 0.8
            detected = self.create_move_top_block_view_and_detect(target_block).result()

            if not detected:
                rospy.logerr("single image cube not detected, recomputing cubes")
                self.create_go_vision_head_pose_task().result()
                blocks = self.create_head_vision_processing_on_table().result()
                rospy.logwarn("new detected cubes: " + str(blocks))
                # target_block_index = 0
                blocks_count = len(blocks)
                continue

            # concurrency issue, what if we lock the objectdetection update?
            self.pick_block_on_table_and_place_on_tray(target_block, target_tray).result()

            blocks_count = len(blocks)

            # rospy.logwarn("target block index: " + str(target_block_index))


    @tasync("DISABLE ROBOT")
    def robot_say(self, msg):
        rospy.logwarn("ROBOT SAYS: "+ str(msg))

    @tasync("DISABLE ROBOT")
    def disable_robot_task(self):
        """
        :return: 
        """
        self.sawyer_robot.disable()

    @tasync("ENABLE ROBOT")
    def enable_robot_task(self):
        """
        :return: 
        """
        self.sawyer_robot.enable()

    @tasync("LOOP SORTING TASK")
    def create_main_loop_task(self):
        """
        This is the main plan of the application
        :return:
        """

        self.create_go_home_task(check_obstacles=False).result()

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

        # self.create_go_home_task().result()

        # self.create_visit_all_cubes_armview(1).result()


        for i in xrange(100):
            self.create_go_vision_head_pose_task().result()
            blocks = self.create_head_vision_processing_on_table().result()

            # self.create_go_home_task().result()

            # self.create_complete_turn_over_tray, target_tray {"homepose": homepose}).result()
            # yield self.create_go_home_task()
            # self.reset_cycle()
            # continue

            self.create_move_all_cubes_to_trays(blocks).result()

            # self.reset_cycle()
            self.robot_say("I FINISHED SORTING THE BLOCKS").result()
            rospy.sleep(3)
            self.robot_say("NOW I WILL PUT THEM BACK ON THE TABLE").result()

            self.pick_all_pieces_from_tray_and_put_on_table().result()

            self.delay_task(1).result()


        self.create_wait_forever_task().result()

    def reset_cycle(self):
        """

        :return: 
        """
        for tray in self.environment_estimation.get_trays():
            tray.reset()

    def get_state(self):
        """
        :return: 
        """
        return {"table_state": self.environment_estimation.table.get_state(),
                "trays": [t.get_state() for t in self.environment_estimation.trays],
                "current_task": self.get_task_stack()}

    def get_task_stack(self):
        return [t.name for t in self.tasks]

    def execute_task(self, fn, args=[]):
        """
        :param fn: 
        :return: 
        """
        # INTERRUPT HERE CURRENT TASK AND SAVE STATE

        self.stop()

        fn(*args).result()

        self.robot_sayt2s("REQUESTED TASK COMPLETED")

        self.delay_task(1)

        # self.create_main_loop_task().result()

    def pause(self):
        """
        :return: 
        """
        rospy.logwarn("PAUSING TASK PLANNER")
        self.pause_flag = True

    def resume(self):
        """
        :return: 
        """
        rospy.logwarn("RESUMING TASK PLANNER")
        self.pause_flag = False

    def stop(self):
        """
        :return: 
        """
        self.sawyer_robot.disable()
        self.sawyer_robot.enable()

        # self.disable_robot_task().result()
        # self.enable_robot_task().result()

        self.cancel_signal = True

        rospy.logwarn("cancelling all tasks")

        try:
            self.mutex.acquire()
            for task in self.tasks:
                rospy.logwarn("cancelling task: " + task.name)
                task.cancel()
        finally:
            self.mutex.release()

        # wait until all tasks are finished
        while len(self.tasks) > 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)

        self.print_tasks()

        self.cancel_signal = False

    def print_tasks(self):
        """
        :return: 
        """
        try:
            self.mutex.acquire()
            tasksstr = "\n".join([str(t.name) for t in self.tasks])
            rospy.logwarn("---- \ntasks stack: \n" + tasksstr + "\n----")
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
