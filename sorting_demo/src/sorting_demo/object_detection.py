import rospy
import tf
import tf.transformations
from gazebo_msgs.msg import LinkStates

import utils
from utils import mathutils
from utils.mathutils import *

class EnvironmentEstimation:
    def __init__(self):
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

            if "block" in name:
                blocks.append((name, pose))
                broadcast = True
            elif "tray" in name:
                trays.append((name, pose))
                broadcast = True

        self.gazebo_blocks = blocks
        self.gazebo_trays = trays

    def update(self):
        # publish tfs
        basehomopose = get_homo_matrix_from_pose_msg(self.gazebo_world_to_ros_transform, tag="base")

        rospy.loginfo("basehomo: " + str(basehomopose))

        collections = [self.gazebo_blocks, self.gazebo_trays]

        blocks = []
        trays = []

        for items in collections:
            for (name, pose) in items:
                # block homogeneous transform
                homopose = get_homo_matrix_from_pose_msg(pose, tag="block")

                transfhomopose = inverse_compose(basehomopose, homopose)

                trans = tf.transformations.translation_from_matrix(transfhomopose)
                quat = tf.transformations.quaternion_from_matrix(transfhomopose)

                self.tf_broacaster.sendTransform(trans,
                                                 quat,
                                                 rospy.Time.now(),
                                                 name,
                                                 "world")

                blocks.append((name, homotransform_to_pose_msg(transfhomopose)))

        self.blocks = blocks

    def get_blocks(self):
        """
        :return array of (name, geometry_msgs.msg.Pose)
        """
        return self.blocks

    def get_trays(self):
        """
        :return array of (name, geometry_msgs.msg.Pose)
        """
        return self.trays
