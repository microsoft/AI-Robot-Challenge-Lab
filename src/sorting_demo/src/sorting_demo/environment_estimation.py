#!/usr/bin/python
import copy
import re

import math
import rospy
import tf
import tf.transformations
from cv_bridge import CvBridge
from gazebo_msgs.msg import LinkStates, sys
from geometry_msgs.msg import Pose, Point, Quaternion
from concepts.block import BlockState
from concepts.tray import TrayState
from concepts.table import Table


from cv_detection_head import CameraHelper, get_blobs_info

from cv_detection_right_hand import get_cubes_z_rotation
from utils.mathutils import *
import demo_constants
from threading import RLock
import cv2


class EnvironmentEstimation:
    def __init__(self):
        """
        """
        self.gazebo_trays = []
        self.gazebo_blocks = []

        self.trays = []
        self.blocks = []

        self.tf_broacaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        # initial simulated implementation
        pub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.simulated_link_state_callback, queue_size=10)

        self.gazebo_world_to_ros_transform = None
        self.original_blocks_poses_ = None
        self.mutex = RLock()

        TABLE_HEIGHT = -0.12
        self.head_camera_helper = CameraHelper("head_camera", "base", TABLE_HEIGHT)
        self.bridge = CvBridge()
        self.block_pose_estimation_head_camera = None
        self.table = Table()

        self.hand_camera_helper = CameraHelper("right_hand_camera", "base", TABLE_HEIGHT)

    def identify_block_from_aproximated_point(self, projected):
        """
        :param projected: 
        :return: 
        """
        bestindex = -1
        bestdist = sys.float_info.max
        for index, b in enumerate(self.blocks):
            p1 = b.pose.position
            p2 = projected
            dx = p1.x - p2[0]
            dy = p1.y - p2[1]
            dz = p1.z - p2[2]

            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            if dist < bestdist:
                bestdist = dist
                bestindex = index

        if bestindex != -1:
            return self.blocks[bestindex]
        else:
            return None

    def compute_block_pose_estimation_from_arm_camera(self, CUBE_SIZE=150):
        #get latest image from topic
        rospy.sleep(0.3)
        # Take picture
        img_data = self.hand_camera_helper.take_single_picture()

        # Convert to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(img_data, "bgr8")

        camwtrans = None
        camwrot = None

        try:
            (camwtrans, camwrot) = self.tf_listener.lookupTransform('/right_hand_camera_optical', '/base',
                                                                    rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
            rospy.logerr(ex.message)

        rotmat = tf.transformations.quaternion_matrix(camwrot)
        transmat = tf.transformations.translation_matrix((camwtrans))
        zaxis = rotmat[2, :]

        cameratransform = tf.transformations.concatenate_matrices(rotmat, transmat)

        # empirically obtained with a drawing, and rviz... the optical z axis is the one that is looking fordware (world x)
        # also weirdly the frame axis in the rotation matrix are rows instead of columns
        camera_yaw_angle = math.atan2(zaxis[1], zaxis[0])

        rospy.logwarn("camera angle:" + str(camera_yaw_angle * 180.0 / math.pi))
        rospy.logwarn("camera rot:" + str(rotmat))
        rospy.logwarn("zaxis camera vector:" + str(zaxis))
        # Save for debugging
        # cv2.imwrite("/tmp/debug.png", cv_image)

        # Get cube rotation
        detected_cubes_info = get_cubes_z_rotation(cv_image, CUBE_SIZE=CUBE_SIZE)
        center = (cv_image.shape[1] / 2, cv_image.shape[0] / 2)

        def cubedistToCenter(cube):
            # ((370, 224), 26.0, True, False)
            dx = cube[0][0] - center[0]
            dy = cube[0][1] - center[1]

            return dx * dx + dy * dy

        sorted_center_cubes = sorted(detected_cubes_info, key=cubedistToCenter)

        try:
            cube = sorted_center_cubes[0]

            image_cube_angle = cube[1] * (math.pi / 180.0)
            graspA = cube[2]
            graspB = cube[3]

            rospy.logwarn("image detected cube angle: " + str(image_cube_angle))

            final_cube_yaw_angle = camera_yaw_angle - image_cube_angle

            while final_cube_yaw_angle > math.pi / 4:
                final_cube_yaw_angle -= math.pi / 2

            while final_cube_yaw_angle < -math.pi / 4:
                final_cube_yaw_angle += math.pi / 2

            # select the other grasping
            if not graspA and graspB:
                rospy.logwarn("Swiching grasping orientation for current block (grasping clearance)")
                if final_cube_yaw_angle >0:
                    final_cube_yaw_angle -= math.pi / 2
                else:
                    final_cube_yaw_angle += math.pi / 2

            projected = self.hand_camera_helper.project_point_on_table(cube[0])
            poseq = tf.transformations.quaternion_from_euler(0, 0, final_cube_yaw_angle)

            rospy.logwarn("quaternion angle:" + str(poseq))
            self.tf_broacaster.sendTransform(projected, poseq, rospy.Time(0), "estimated_cube_1", "base")
            rospy.logwarn(projected)

            #cv2.imshow("cube detection", cv_image)
            #cv2.waitKey(0)


            return Pose(position=Point(x=projected[0], y=projected[1], z=projected[1]),
                        orientation=Quaternion(x=poseq[0], y=poseq[1], z=poseq[2], w=poseq[3])),graspA or graspB
        except Exception as ex:
            rospy.logwarn("erroneous cube detection")
            #cv2.imshow("erroneus cube detection", cv_image)
            #cv2.waitKey(0)
            return None, False

    def compute_block_pose_estimations_from_head_camera(self):
        """
        For each block updates:
            - block.headview_pose_estimation
            - block.hue_estimation
        :return: 
        """
        try:
            self.mutex.acquire()
            img_data = self.head_camera_helper.take_single_picture()

            # Convert to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(img_data, "bgr8")
            cv2.imwrite("/tmp/last_head_picture.jpg",cv_image)

            rospy.logwarn("processing head camera image to find blocks")
            blobs_info = get_blobs_info(cv_image)

            index = 0
            ptinfos = []
            for huekey in blobs_info.keys():
                points = blobs_info[huekey]
                rospy.logwarn("blob position[%d]: %s" % (index, str(points)))
                for point in points:
                    ptinfos.append([huekey, point])
                    index += 1

            detected_blocks = []

            for huekey, point2d in ptinfos:
                projected = self.head_camera_helper.project_point_on_table(point2d)
                rospy.logwarn("projected: %s" % str(projected))

                block = self.identify_block_from_aproximated_point(projected)
                if block is None:
                    continue

                detected_blocks.append(block)

                block.headview_proj_estimation = point2d

                block.headview_proj_estimation = projected
                block.hue_estimation = huekey
                block.headview_pose_estimation = Pose(
                    position=Point(x=projected[0], y=projected[1], z=projected[2]),
                    orientation=Quaternion(x=0, y=0, z=0, w=1))

                rospy.logwarn("blob identified: " + str(block))

            rospy.logwarn("Table blocks:")
            self.table.blocks = detected_blocks
            for b in self.table.blocks:
                rospy.logwarn(b)

            self.blocks = self.table.blocks

            for tray in self.trays:

                rospy.logwarn("Tray blocks:")
                for b in tray.blocks:
                    rospy.logwarn(b)

                self.blocks = self.blocks + tray.blocks

            rospy.logwarn("All known blocks:")
            for b in self.blocks:
                rospy.logwarn(b)
        finally:
            self.mutex.release()

    def simulated_link_state_callback(self, links):
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
                    item = self.get_block_by_gazebo_id(name)
                    if item is None:
                        # rospy.logwarn("block create name: "+ name)
                        item = BlockState(gazebo_id=name, pose=pose)
                        item.color = demo_constants.BLOCK_COLOR_MAPPINGS[item.num]["material"].replace("Gazebo/", "")
                    else:
                        item.pose = pose

                    blocks.append(item)
                elif TrayState.is_tray(name):
                    item = self.get_tray_by_gazebo_id(name)
                    # item = None
                    if item is None:
                        item = TrayState(gazebo_id=name, pose=pose, TRAY_SURFACE_THICKNESS= demo_constants.TRAY_SURFACE_THICKNESS)
                        item.color = demo_constants.TRAY_COLORS[item.num].replace("Gazebo/", "")
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
                                                     item.gazebo_id,
                                                     "world")

                    item.gazebo_pose = homotransform_to_pose_msg(transf_homopose)

                    if isinstance(item, BlockState):
                        blocks.append(item)
                    elif isinstance(item, TrayState):
                        trays.append(item)
                        # else:
                        # rospy.logwarn("DETECTED ITEM:" + str(item))

            self.blocks = blocks
            self.trays = trays

            # rospy.logwarn("GAZEBO blocks update lenght: %d"%len(self.gazebo_blocks))
            # rospy.logwarn("blocks update lenght: %d"%len(self.blocks))
            if self.original_blocks_poses_ is None:
                self.original_blocks_poses_ = [copy.deepcopy(block.gazebo_pose) for block in blocks]

        finally:
            self.mutex.release()

    def get_blocks(self):
        """
        :return array of (name, geometry_msgs.msg.Pose)
        """
        try:
            self.mutex.acquire()
            return [b for b in self.blocks]
        finally:
            self.mutex.release()

    def get_block_by_gazebo_id(self, gazebo_id):
        """
        :param gazebo_id:
        :return:
        """
        try:
            self.mutex.acquire()

            filtered_blocks = [block for block in self.blocks if block.gazebo_id == gazebo_id]
            if len(filtered_blocks) == 0:
                return None
            else:
                return filtered_blocks[0]
        finally:
            self.mutex.release()

    def get_original_block_poses(self):
        """
        :return: 
        """
        try:
            self.mutex.acquire()
            return [copy.deepcopy(p) for p in self.original_blocks_poses_]
        finally:
            self.mutex.release()

    def get_tray_by_gazebo_id(self, gazebo_id):
        """
        :param gazebo_id:
        :return:
        """
        try:
            self.mutex.acquire()
            filtered_trays = [tray for tray in self.trays if tray.gazebo_id == gazebo_id]
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
