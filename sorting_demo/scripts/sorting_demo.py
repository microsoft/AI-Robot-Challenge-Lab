#!/usr/bin/env python

import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import intera_interface


class SortingDemo(object):
    def __init__(self, limb="right", hover_distance=0.15, tip_name="right_gripper_tip"):
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
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0] * 7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()

    def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
        if rospy.is_shutdown():
            return
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles, timeout=timeout)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self._limb.ik_request(approach, self._tip_name)
        self._limb.set_joint_position_speed(0.001)
        self._guarded_move_to_joint_position(joint_angles)
        self._limb.set_joint_position_speed(0.1)

    def _retract(self):
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
        self._servo_to_pose(ik_pose)

    def _servo_to_pose(self, pose, time=4.0, steps=400.0):
        ''' An *incredibly simple* linearly-interpolated Cartesian move '''
        r = rospy.Rate(1 / (time / steps))  # Defaults to 100Hz command rate
        current_pose = self._limb.endpoint_pose()
        ik_delta = Pose()
        ik_delta.position.x = (current_pose['position'].x - pose.position.x) / steps
        ik_delta.position.y = (current_pose['position'].y - pose.position.y) / steps
        ik_delta.position.z = (current_pose['position'].z - pose.position.z) / steps
        ik_delta.orientation.x = (current_pose['orientation'].x - pose.orientation.x) / steps
        ik_delta.orientation.y = (current_pose['orientation'].y - pose.orientation.y) / steps
        ik_delta.orientation.z = (current_pose['orientation'].z - pose.orientation.z) / steps
        ik_delta.orientation.w = (current_pose['orientation'].w - pose.orientation.w) / steps
        for d in range(int(steps), -1, -1):
            if rospy.is_shutdown():
                return
            ik_step = Pose()
            ik_step.position.x = d * ik_delta.position.x + pose.position.x
            ik_step.position.y = d * ik_delta.position.y + pose.position.y
            ik_step.position.z = d * ik_delta.position.z + pose.position.z
            ik_step.orientation.x = d * ik_delta.orientation.x + pose.orientation.x
            ik_step.orientation.y = d * ik_delta.orientation.y + pose.orientation.y
            ik_step.orientation.z = d * ik_delta.orientation.z + pose.orientation.z
            ik_step.orientation.w = d * ik_delta.orientation.w + pose.orientation.w
            joint_angles = self._limb.ik_request(ik_step, self._tip_name)
            if joint_angles:
                self._limb.set_joint_positions(joint_angles)
            else:
                rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
            r.sleep()
        rospy.sleep(1.0)

    def pick(self, pose):
        if rospy.is_shutdown():
            return
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        if rospy.is_shutdown():
            return
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        if rospy.is_shutdown():
            return
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        if rospy.is_shutdown():
            return
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()


def load_gazebo_models(table_pose=Pose(position=Point(x=0.75, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('sawyer_sim_examples') + "/models/"
    # Load Table SDF
    table_xml = ''
    with open(model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml = table_file.read().replace('\n', '')
    # Load Block URDF
    block_xml = ''
    with open(model_path + "block/model.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))


def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        print("Delete Model service call failed: {0}".format(e))


def main():
    """SDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Sawyer will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    rospy.init_node("ik_pick_and_place_demo")
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    load_gazebo_models()
    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)

    limb = 'right'
    hover_distance = 0.15  # meters
    # Starting Joint angles for right arm
    starting_joint_angles = {'right_j0': -0.041662954890248294,
                             'right_j1': -1.0258291091425074,
                             'right_j2': 0.0293680414401436,
                             'right_j3': 2.17518162913313,
                             'right_j4': -0.06703022873354225,
                             'right_j5': 0.3968371433926965,
                             'right_j6': 1.7659649178699421}
    pnp = SortingDemo(limb, hover_distance)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
        x=-0.00142460053167,
        y=0.999994209902,
        z=-0.00177030764765,
        w=0.00253311793936)
    block_poses = list()
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.
    block_poses.append(Pose(
        position=Point(x=0.45, y=0.155, z=-0.129),
        orientation=overhead_orientation))
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    block_poses.append(Pose(
        position=Point(x=0.6, y=-0.1, z=-0.129),
        orientation=overhead_orientation))
    # Move to the desired starting angles
    print("Running. Ctrl-c to quit")
    pnp.move_to_start(starting_joint_angles)
    idx = 0
    while not rospy.is_shutdown():
        print("\nPicking...")
        pnp.pick(block_poses[idx])
        print("\nPlacing...")
        idx = (idx + 1) % len(block_poses)
        pnp.place(block_poses[idx])
    return 0


if __name__ == '__main__':
    sys.exit(main())
