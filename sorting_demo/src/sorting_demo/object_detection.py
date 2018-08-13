#!/usr/bin/python
import re

import rospy
import tf.transformations
from gazebo_msgs.msg import LinkStates

from utils.mathutils import *


class TrayState:
    regex = re.compile(r'tray(\d+)\.*')

    def __init__(self, id, pose):
        self.id = id
        self.pose = pose
        self.homogeneous_transform = None

        search = TrayState.regex.search(id)
        self.num = search.group(1)

    @staticmethod
    def is_tray(id):
        num = TrayState.regex.search(id)
        return num is not None


class BlockState:
    regex = re.compile(r'block(\d+)\.*')

    def __init__(self, id, pose):
        self.id = id
        self.pose = pose
        self.homogeneous_transform = None
        search = BlockState.regex.search(id)
        self.num = search.group(1)

    @staticmethod
    def is_block(id):
        num = BlockState.regex.search(id)
        return num is not None


class EnvironmentEstimation:
    def __init__(self):
        """

        """
        self.gazebo_trays = []
        self.gazebo_blocks = []

        self.trays = []
        self.blocks = []

        self.tf_broacaster = tf.TransformBroadcaster()

        # initial simulated implementation
        pub = rospy.Subscriber('/gazebo/link_states', LinkStates, self._links_callback, queue_size=10)

        self.gazebo_world_to_ros_transform = None

    def _links_callback(self, links):
        """
        string[] name
        geometry_msgs/Pose[] pose
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
        geometry_msgs/Twist[] twist
          geometry_msgs/Vector3 linear
            float64 x
            float64 y
            float64 z
          geometry_msgs/Vector3 angular
            float64 x
            float64 y
            float64 z

        :param links: 
        :return: 
        """
        blocks = []
        trays = []

        base_index = [i for i, name in enumerate(links.name) if name == "sawyer::base"][0]
        self.gazebo_world_to_ros_transform = links.pose[base_index]

        for i, name in enumerate(links.name):
            pose = links.pose[i]
            broadcast = False

            item = None
            if BlockState.is_block(name):
                item = BlockState(id=name, pose=pose)
                blocks.append(item)
            elif TrayState.is_tray(name):
                item = TrayState(id=name, pose=pose)
                trays.append(item)
            else:
                continue

            #rospy.logwarn("simulated object state: " + name + " -> " + item.num)

        self.gazebo_blocks = blocks
        self.gazebo_trays = trays

    def update(self):
        """

        :return:
        """
        # publish tfs
        basehomopose = get_homo_matrix_from_pose_msg(self.gazebo_world_to_ros_transform, tag="base")

        # rospy.logwarn("basehomo: " + str(basehomopose))

        collections = [self.gazebo_blocks, self.gazebo_trays]

        blocks = []
        trays = []

        for items in collections:
            for item in items:

                # block homogeneous transform
                homo_pose = get_homo_matrix_from_pose_msg(item.pose, tag="block")

                # rospy.logwarn("TF PUBLISH..." +  str(homo_pose))
                # rospy.logwarn("item state: " + str(item))

                transf_homopose = inverse_compose(basehomopose, homo_pose)

                trans = tf.transformations.translation_from_matrix(transf_homopose)
                quat = tf.transformations.quaternion_from_matrix(transf_homopose)

                self.tf_broacaster.sendTransform(trans,
                                                 quat,
                                                 rospy.Time.now(),
                                                 item.id,
                                                 "world")

                item.final_pose = homotransform_to_pose_msg(transf_homopose)

                if isinstance(item, BlockState):
                    blocks.append(item)
                elif isinstance(item, TrayState):
                    trays.append(item)
                #else:
                    #rospy.logwarn("DETECTED ITEM:" + str(item))

        self.blocks = blocks
        self.trays = trays

    def get_blocks(self):
        """
        :return array of (name, geometry_msgs.msg.Pose)
        """
        return self.blocks

    def get_tray(self, id):
        """

        :param id:
        :return:
        """
        return [tray for tray in self.trays if tray.id == id][0]

    def get_trays(self):
        """
        :return array of (name, geometry_msgs.msg.Pose)
        """
        return self.trays
