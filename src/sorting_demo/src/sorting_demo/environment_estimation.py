#!/usr/bin/python
import copy
import re

import math
import rospy
import tf.transformations
from cv_bridge import CvBridge
from gazebo_msgs.msg import LinkStates, sys

from concepts.block import BlockState
from concepts.tray import TrayState
from cv_detection import CameraHelper, get_blobs_info

from utils.mathutils import *
import demo_constants
from threading import RLock


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
        self.original_blocks_poses_ = None
        self.mutex = RLock()

        camera_name = "head_camera"
        TABLE_HEIGHT = -0.12
        self.camera_helper = CameraHelper(camera_name, "base", TABLE_HEIGHT)
        self.bridge = CvBridge()

        self.block_pose_estimation_head_camera = None

    def identify_block_by_pos(self, projected):
        bestindex = -1
        bestdist = sys.float_info.max
        for index, b in enumerate(self.blocks):
            p1 = b.pose.position
            p2 = projected
            dx = p1.x - p2[0]
            dy = p1.y - p2[1]
            dz = p1.z - p2[2]

            dist = math.sqrt(dx*dx + dy*dy + dz*dz)

            if dist < bestdist:
                bestdist = dist
                bestindex = index

        if bestindex != -1:
            return self.blocks[bestindex]
        else:
            return None



    def compute_block_pose_estimations_from_head_camera(self):
        try:
            self.mutex.acquire()
            img_data = self.camera_helper.take_single_picture()

            # Convert to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(img_data, "bgr8")

            blobs_info = get_blobs_info(cv_image)

            index = 0
            ptinfos = []
            for huekey in blobs_info.keys():
                points = blobs_info[huekey]
                rospy.logwarn("blob position[%d]: %s"%(index , str(points)))
                for point in points:
                    ptinfos.append([huekey, point])
                    index+=1

            for huekey, point2d in ptinfos:
                projected = self.camera_helper.project_point_on_table(point2d)
                rospy.logwarn("projected: %s" % str(projected))

                blob = self.identify_block_by_pos(projected)
                if blob is None:
                    continue

                blob.headview_proj_estimation = point2d

                blob.headview_proj_estimation = projected
                blob.hue_estimation = huekey
                blob.headview_pose_estimation = Pose(
                    position=Point(x=projected[0], y=projected[1], z=projected[2]),
                    orientation=Quaternion(x=0, y=0, z=0, w=1))

                rospy.logwarn("blob identified: "+ str(blob))

            for b in self.blocks:
                rospy.logwarn(b)
        finally:
            self.mutex.release()

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
        try:
            self.mutex.acquire()
            blocks = []
            trays = []

            base_index = [i for i, name in enumerate(links.name) if name == "sawyer::base"][0]
            self.gazebo_world_to_ros_transform = links.pose[base_index]

            for i, name in enumerate(links.name):
                pose = links.pose[i]

                if BlockState.is_block(name):
                    item = self.get_block(name)
                    if item is None:
                        #rospy.logwarn("block create name: "+ name)
                        item = BlockState(id=name, pose=pose)
                        item.color = demo_constants.BLOCK_COLOR_MAPPINGS[item.num]["material"]
                    else:
                        item.pose = pose

                    blocks.append(item)
                elif TrayState.is_tray(name):
                    item = self.get_tray(name)
                    # item = None
                    if item is None:
                        item = TrayState(id=name, pose=pose)
                        item.color = demo_constants.TRAY_COLORS[item.num]
                    else:
                        item.pose = pose

                    trays.append(item)
                else:
                    continue

                    # rospy.logwarn("simulated object state: " + name + " -> " + item.num)

            self.gazebo_blocks = blocks
            self.gazebo_trays = trays
        except Exception as ex:
            rospy.logerr(ex.message)
        finally:
            self.mutex.release()

    def update(self):
        """
        this method basically double buffers the state of block and trays. It also publishes tf.
        For the simulated case it copies from gazebo_blocks and gazebo_trays to blocks and trays
        :return:
        """

        try:
            self.mutex.acquire()

            collections = [self.gazebo_blocks, self.gazebo_trays]

            blocks = []
            trays = []

            # publish tfs
            basehomopose = get_homo_matrix_from_pose_msg(self.gazebo_world_to_ros_transform, tag="base")

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
                        # else:
                        # rospy.logwarn("DETECTED ITEM:" + str(item))

            self.blocks = blocks
            self.trays = trays

            #rospy.logwarn("GAZEBO blocks update lenght: %d"%len(self.gazebo_blocks))
            #rospy.logwarn("blocks update lenght: %d"%len(self.blocks))
            if self.original_blocks_poses_ is None:
                self.original_blocks_poses_ = [copy.deepcopy(block.final_pose) for block in blocks]

        finally:
            self.mutex.release()

    def get_blocks(self):
        """
        :return array of (name, geometry_msgs.msg.Pose)
        """
        try:
            self.mutex.acquire()
            return [copy.deepcopy(b) for b in self.blocks]
        finally:
            self.mutex.release()

    def get_block(self, id):
        """

        :param id:
        :return:
        """
        try:
            self.mutex.acquire()

            filtered_blocks = [block for block in self.blocks if block.id == id]
            if len(filtered_blocks) == 0:
                return None
            else:
                return filtered_blocks[0]
        finally:
            self.mutex.release()


    def get_original_block_poses(self):
        try:
            self.mutex.acquire()
            return [copy.deepcopy(p) for p in self.original_blocks_poses_]
        finally:
            self.mutex.release()

    def get_tray(self, id):
        """

        :param id:
        :return:
        """
        try:
            self.mutex.acquire()
            filtered_trays = [tray for tray in self.trays if tray.id == id]
            if len(filtered_trays) == 0:
                return None
            else:
                return filtered_trays[0]
        finally:
            self.mutex.release()

    def get_tray_by_color(self, color):
        """

        :param id:
        :return:
        """

        color = color.replace("Gazebo/", "")
        rospy.logwarn("by color: " + str(color))
        rospy.logwarn("by color: " + str(self.trays))

        filtered_trays = [tray for tray in self.trays if tray.color == color]
        if len(filtered_trays) == 0:
            return None
        else:
            return filtered_trays[0]

    def get_tray_by_num(self, num):
        """
        
        :param num: 
        :return: 
        """
        rospy.logwarn("by num: " + str(num))
        rospy.logwarn("by trays: " + str([t.num for t in self.trays]))

        filtered_trays = [tray for tray in self.trays if int(tray.num) == int(num)]
        if len(filtered_trays) == 0:
            return None
        else:
            return filtered_trays[0]

    def get_trays(self):
        """
        :return array of (name, geometry_msgs.msg.Pose)
        """
        return self.trays
