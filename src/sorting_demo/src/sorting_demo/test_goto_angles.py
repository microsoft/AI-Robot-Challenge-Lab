#!/usr/bin/env python
import random
from rospkg.distro import current_distro_codename

import rospy
import argparse
from geometry_msgs.msg import Pose
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions,
    InteractionOptions
)
import trajectory_msgs.msg
from intera_motion_msgs.msg import TrajectoryOptions
from intera_interface import Limb
from intera_motion_interface.utility_functions import int2bool
import moveit_msgs.msg
import actionlib
import control_msgs.msg
import numpy
import math
import copy

from geometry_msgs.msg import Pose, Point, Quaternion


def iterative_ik_find(limb, approach_pose, tipname):
    computed_joints = False
    joint_seed = copy.deepcopy(limb.joint_angles())

    while computed_joints is False and not rospy.is_shutdown():

        for k in joint_seed.keys():
            joint_seed[k] = joint_seed[k] + random.uniform(-0.2 * math.pi / 2, 0.2 * math.pi / 2)

        #rospy.logwarn("JOINT SEED:" + str(joint_seed))
        joint_seed = limb.joint_angles()
        computed_joints = limb.ik_request(approach_pose, tipname,joint_seed=joint_seed)

        #rospy.logwarn("APPROACH JOINTS: " + str(approach_joints))

        if computed_joints is False:
            rospy.logerr(approach_pose)
            joint_seed = copy.deepcopy(limb.joint_angles())
        else:
            joint_seed = copy.deepcopy(computed_joints)

    return computed_joints

def execute_linear_motion(limb, target_pose, steps, tipname, total_time_sec):
    ik_delta = Pose()

    current_pose = limb.endpoint_pose()
    joint_angles_trajectory = []

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

        joint_angles = iterative_ik_find(limb, ik_step,tipname)
        if joint_angles:
            joint_angles_trajectory.append(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    execute_joint_trajectory(limb, joint_angles_trajectory, total_time_sec)



def fill_velocities_and_accelerations(limb, goal, names, dt):

    #velocities
    for i in xrange(len(goal.goal.trajectory.points) -1):
        p1 = goal.goal.trajectory.points[i]
        p2 = goal.goal.trajectory.points[i+1]
        for j in xrange(len(names)):
            v = (p2.positions[j] - p1.positions[j] )/ dt
            goal.goal.trajectory.points[i].velocities.append(v)

    for j in xrange(len(names)):
        goal.goal.trajectory.points[-1].velocities.append(0)

    #accelerations
    for i in xrange(len(goal.goal.trajectory.points) - 1):
        p1 = goal.goal.trajectory.points[i]
        p2 = goal.goal.trajectory.points[i + 1]
        for j in xrange(len(names)):
            a = (p2.velocities[j] - p1.velocities[j]) / dt
            goal.goal.trajectory.points[i].accelerations.append( a)

    for j in xrange(len(names)):
        goal.goal.trajectory.points[-1].accelerations.append( 0)

    # goal.goal.trajectory.joint_trajectory.header.seq =
    goal.goal.trajectory.header.stamp = rospy.Time.now()
    goal.goal.trajectory.joint_names = limb.joint_names()

    rospy.logwarn(goal.goal.trajectory.joint_names)

def execute_joint_trajectory(limb, target_joint_trajectory, total_time_sec):
    """
    :param limb: 
    :param target_joints: float[]
    :return: 
    """
    #rospy.init_node('go_to_joint_angles_in_contact_py')
    names = limb.joint_names()
    #initial_joint_values = limb.joint_angles()
    initial_joint_values = target_joint_trajectory[0]

    """
    target_joints[0]=1
    target_joints[1]=-1
    target_joints[2]=1
    target_joints[3]=1
    target_joints[4]=1
    target_joints[5]=1
    target_joints[6]=1
    """

    rospy.logwarn("initial: " + str(initial_joint_values))

    client = actionlib.SimpleActionClient('/robot/limb/right/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)

    client.wait_for_server()

    goal = control_msgs.msg.FollowJointTrajectoryActionGoal()
    goal.header.stamp=rospy.Time.now()

    steps = len(target_joint_trajectory)
    total_time_sec = float(total_time_sec)
    dt = total_time_sec/steps
    t = 0

    for  i in xrange(len (target_joint_trajectory)):
        t+= dt
        current_joints = []
        #for i,name in enumerate(names):
        #    current_joints.append(initial_joint_values[name] * (1 - t) + target_joints[i] * t)

        current_joints = target_joint_trajectory[i]
        rospy.logwarn("points i"+ str(current_joints))
        pt = trajectory_msgs.msg.JointTrajectoryPoint()

        pt.positions = [current_joints[name] for name in names]
        pt.time_from_start = rospy.Duration(t*total_time_sec)

        goal.goal.trajectory.points.append(pt)

    fill_velocities_and_accelerations(limb, goal,names, dt)

    """
    #joint tolerances
    for name in goal.goal.trajectory.joint_names :
        tolerance = control_msgs.msg.JointTolerance()
        tolerance.name = name
        tolerance.position = 0.2
        goal.goal.path_tolerance.append(tolerance)
    
    
    #goal tolerances
    for name in goal.goal.trajectory.joint_names :
        tolerance = control_msgs.msg.JointTolerance()
        tolerance.name = name
        tolerance.position = 0.05
        goal.goal.goal_tolerance.append(tolerance)
    """


    # Sends the goal to the action server.
    client.send_goal(goal.goal)

    #while client.get_state() == actionlib.GoalStatus.ACTIVE and not rospy.is_shutdown():
    #    rospy.sleep(0.1)
    # Waits for the server to finish performing the action.
    success = client.wait_for_result()

    # Prints out the result of executing the action
    rospy.logwarn(client.get_result())  # A FibonacciResult
    return success

"""
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
actionlib_msgs/GoalID goal_id
  time stamp
  string id
control_msgs/FollowJointTrajectoryGoal goal
  trajectory_msgs/JointTrajectory trajectory
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    string[] joint_names
    trajectory_msgs/JointTrajectoryPoint[] points
      float64[] positions
      float64[] velocities
      float64[] accelerations
      float64[] effort
      duration time_from_start
  control_msgs/JointTolerance[] path_tolerance
    string name
    float64 position
    float64 velocity
    float64 acceleration
  control_msgs/JointTolerance[] goal_tolerance
    string name
    float64 position
    float64 velocity
    float64 acceleration
  duration goal_time_tolerance

"""

"""
if __name__=="__main__":
    rospy.init_node('go_to_joint_angles_in_contact_py')
    limb = Limb()

    '''
    names = limb.joint_names()
    initial = limb.joint_angles()
    target_joints = [limb.joint_angle(name) for name in names]
    total_time_sec = 10.0

    go_to_joints(limb,target_joints,total_time_sec=total_time_sec)
    '''

    current_pose = limb.endpoint_pose()

    current_pose2 =Pose(position=Point(x=current_pose['position'].x, y= current_pose['position'].y, z= current_pose['position'].z),orientation=Quaternion(x= current_pose['orientation'].x, y=current_pose['orientation'].y,z=current_pose['orientation'].z,w=current_pose['orientation'].w))
    current_pose2.position.z-=0.2


    execute_linear_motion(limb, current_pose2, steps=1000, tipname="right_gripper_tip", total_time_sec=5.0)

    rospy.spin()
"""