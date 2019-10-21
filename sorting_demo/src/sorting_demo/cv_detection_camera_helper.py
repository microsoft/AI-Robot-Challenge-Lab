#!/usr/bin/python
import Queue
import numpy

import rospy
import tf

from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel

from visualization_msgs.msg import *

import intera_interface

class CameraHelper:
    """
    A helper class to take pictures with the Sawyer camera and unproject points from the images
    """
    def __init__(self, camera_name, base_frame):
        """
        Initialize the instance

        :param camera_name: The camera name. One of ("head_camera", "right_hand_camera")
        :param base_frame: The frame for the robot base
        """
        self.camera_name = camera_name
        self.base_frame = base_frame

        self.image_queue = Queue.Queue()
        self.pinhole_camera_model = PinholeCameraModel()
        self.tf_listener = tf.TransformListener()

        camera_info_topic = "/io/internal_camera/{}/camera_info".format(camera_name)
        camera_info = rospy.wait_for_message(camera_info_topic, CameraInfo)

        self.pinhole_camera_model.fromCameraInfo(camera_info)

        self.cameras = intera_interface.Cameras()
        self.cameras.set_callback(camera_name, self.__show_image_callback, rectify_image=True)

    def __show_image_callback(self, img_data):
        """
        Image callback for the camera

        :param img_data: The image received from the camera
        """
        if self.image_queue.empty():
            self.image_queue.put(img_data)

    def set_exposure(self, exposure):
        self.cameras.set_exposure(self.camera_name, exposure)

    def set_gain(self, gain):
        self.cameras.set_gain(self.camera_name, gain)

    def set_cognex_strobe(self, value):
        self.cameras.set_cognex_strobe(value)

    def take_single_picture(self):
        """
        Take one picture from the specified camera

        :returns: The image
        """
        with self.image_queue.mutex:
            self.image_queue.queue.clear()

        cameras = intera_interface.Cameras()

        cameras.start_streaming(self.camera_name)

        image_data = self.image_queue.get(block=True, timeout=None)

        cameras.stop_streaming(self.camera_name)

        return image_data

    def project_point_on_table(self, point, table_height):
        """
        Projects the 2D point from the camera image on the table

        :param point: The 2D point in the form (x, y)
        :return: The 3D point in the coordinate space of the frame that was specified when the object was initialized
        """

        # Unproject point
        unprojected_ray_camera = self.pinhole_camera_model.projectPixelTo3dRay(point)

        # Transform ray into base frame
        identity_quaternion = [1.0, 0.0, 0.0, 0.0]
        camera_position, _ = self.transform_pose_to_base_frame([0.0, 0.0, 0.0], identity_quaternion)
        camera_ray_point, _ = self.transform_pose_to_base_frame(unprojected_ray_camera, identity_quaternion)

        # Intersect ray with base plane
        camera_ray_direction = camera_ray_point - camera_position
        point_height = camera_position[2] - table_height
        factor = -point_height / camera_ray_direction[2]
        intersection = camera_position + camera_ray_direction * factor

        return intersection

    def transform_pose_to_base_frame(self, translation, orientation):
        """
        Transforms a translation/orientation pose from the camera frame to the base frame

        :param translation: The translation vector
        :param orientation: The orientation quaternion
        :return: The translation/orientation pose in the base frame coordinate space
        """

        # Get camera frame name
        camera_frame = self.pinhole_camera_model.tfFrame()

        # Check that both frames exist
        if self.tf_listener.frameExists(self.base_frame) and self.tf_listener.frameExists(camera_frame):
            # Get transformation
            time = self.tf_listener.getLatestCommonTime(self.base_frame, camera_frame)

            camera_translation_base, camera_orientation_base = self.tf_listener.lookupTransform(self.base_frame, camera_frame, time)

            # Create base-camera matrix
            camera_base_translation_matrix = tf.transformations.translation_matrix(camera_translation_base)
            camera_base_orientation_matrix = tf.transformations.quaternion_matrix(camera_orientation_base)
            camera_base_matrix = numpy.matmul(camera_base_translation_matrix, camera_base_orientation_matrix)

            # Create camera-pose matrix
            pose_translation_matrix = tf.transformations.translation_matrix(translation)
            pose_orientation_matrix = tf.transformations.quaternion_matrix(orientation)
            pose_matrix = numpy.matmul(pose_translation_matrix, pose_orientation_matrix)

            # Multiply both matrices
            final_matrix = numpy.matmul(camera_base_matrix, pose_matrix)

            # Get final pose
            final_translation = tf.transformations.translation_from_matrix(final_matrix)
            final_orientation = tf.transformations.quaternion_from_matrix(final_matrix)

            return final_translation, final_orientation
