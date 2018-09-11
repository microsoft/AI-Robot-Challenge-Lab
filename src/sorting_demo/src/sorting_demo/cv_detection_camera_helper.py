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
    def __init__(self, camera_name, base_frame, table_height):
        """
        Initialize the instance

        :param camera_name: The camera name. One of (head_camera, right_hand_camera)
        :param base_frame: The frame for the robot base
        :param table_height: The table height with respect to base_frame
        """
        self.camera_name = camera_name
        self.base_frame = base_frame
        self.table_height = table_height

        self.image_queue = Queue.Queue()
        self.pinhole_camera_model = PinholeCameraModel()
        self.tf_listener = tf.TransformListener()

        camera_info_topic = "/io/internal_camera/{}/camera_info".format(camera_name)
        camera_info = rospy.wait_for_message(camera_info_topic, CameraInfo)

        self.pinhole_camera_model.fromCameraInfo(camera_info)

        cameras = intera_interface.Cameras()
        cameras.set_callback(camera_name, self.__show_image_callback, rectify_image=True)

    def __show_image_callback(self, img_data):
        """
        Image callback for the camera

        :param img_data: The image received from the camera
        """
        if self.image_queue.empty():
            self.image_queue.put(img_data)

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

    def project_point_on_table(self, point):
        """
        Projects the 2D point from the camera image on the table

        :param point: The 2D point in the form (x, y)
        :return: The 3D point in the coordinate space of the frame that was specified when the object was initialized
        """

        # Get camera frame name
        camera_frame = self.pinhole_camera_model.tfFrame()

        # Check that both frames exist
        if self.tf_listener.frameExists(self.base_frame) and self.tf_listener.frameExists(camera_frame):
            # Get transformation
            time = self.tf_listener.getLatestCommonTime(self.base_frame, camera_frame)
            camera_translation_base, camera_orientation_base = self.tf_listener.lookupTransform(self.base_frame, camera_frame, time)

            # Unproject point
            unprojected_ray_camera = self.pinhole_camera_model.projectPixelTo3dRay(point)

            # Rotate ray based on the frame transformation
            camera_frame_rotation_matrix = tf.transformations.quaternion_matrix(camera_orientation_base)
            unprojected_ray_base = numpy.dot(camera_frame_rotation_matrix[:3,:3], unprojected_ray_camera)

            # Intersect ray with base plane
            point_height = camera_translation_base[2] - self.table_height
            factor = -point_height / unprojected_ray_base[2]
            intersection = camera_translation_base + unprojected_ray_base * factor

            return intersection
