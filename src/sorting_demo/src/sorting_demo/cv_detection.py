#!/usr/bin/python
import sys
import Queue
import numpy
import os

from matplotlib import pyplot

import rospy
import tf
import cv2
import rospkg

from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge, CvBridgeError

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

def get_blob_info(cv_image):
    """
    Gets information about the colored blobs in the image

    :param cv_image: The OpenCV image to get blobs from
    :return: A dictionary containing an array of points for each colored blob found, represented by its hue.
        For example, in an image with 2 red blobs and 3 blue blobs, it will return
            {0: [(639, 558), (702, 555)], 120: [(567, 550), (698, 515), (648, 515)]}
    """

    # Show original image
    #cv2.imshow("Original image", cv_image)

    # Apply blur
    BLUR_SIZE = 3
    cv_image_blur = cv2.GaussianBlur(cv_image, (BLUR_SIZE, BLUR_SIZE), 0)
    #cv2.imshow("Blur", cv_image_blur)

    # Apply ROI mask
    mask_path = rospkg.RosPack().get_path('sorting_demo') + "/share/head-mask.png"
    cv_mask = cv2.imread(mask_path)
    cv_mask = cv2.cvtColor(cv_mask, cv2.COLOR_BGR2GRAY)

    cv_image_masked = cv2.bitwise_and(cv_image_blur, cv_image_blur, mask = cv_mask)
    cv2.imshow("Masked original", cv_image_masked)

    # HSV split
    cv_image_hsv = cv2.cvtColor(cv_image_masked, cv2.COLOR_BGR2HSV)
    cv_image_h, cv_image_s, cv_image_v = cv2.split(cv_image_hsv)
    #cv2.imshow("Image H", cv_image_h)
    #cv2.imshow("Image S", cv_image_s)
    #cv2.imshow("Image V", cv_image_v)

    # Apply CLAHE to Value channel
    CLAHE_SIZE = 16
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(CLAHE_SIZE, CLAHE_SIZE))
    cv_image_v_clahe = clahe.apply(cv_image_v)

    # Merge channels
    cv_image_clahe = cv2.merge((cv_image_h, cv_image_s, cv_image_v_clahe))
    #cv2.imshow("CLAHE", cv_image_clahe)

    # Multiply Saturation and Value channels to separate the cubes, removing the table
    cv_image_sv_multiplied = cv2.multiply(cv_image_s, cv_image_v_clahe, scale=1/255.0)
    #cv2.imshow("Image S*V", cv_image_sv_multiplied)

    # Binarize the result
    BIN_THRESHOLD = 127
    #ret, cv_image_binary = cv2.threshold(cv_image_sv_multiplied, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    ret, cv_image_binary = cv2.threshold(cv_image_sv_multiplied, BIN_THRESHOLD, 255, cv2.THRESH_BINARY)
    #cv2.imshow("Threshold", cv_image_binary)

    # Calculate H-channel histogram, applying the binarized image as mask
    cv_image_h_histogram = cv2.calcHist([cv_image_h], [0], cv_image_binary, [256], [0, 255])

    # Smoothen the histogram to find local maxima
    HISTOGRAM_BLUR_SIZE = 11
    histogram_length = len(cv_image_h_histogram)
    cv_image_h_histogram_wrap = cv2.repeat(cv_image_h_histogram, 3, 1)
    cv_image_h_histogram_smooth = cv2.GaussianBlur(cv_image_h_histogram_wrap, (HISTOGRAM_BLUR_SIZE, HISTOGRAM_BLUR_SIZE), 0)
    cv_image_h_histogram_cut = cv_image_h_histogram_smooth[histogram_length : 2 * histogram_length]

    # Collect high peaks
    PEAK_RATIO = 0.2
    histogram_peak_cut = PEAK_RATIO * max(cv_image_h_histogram_cut)
    histogram_peaks = [i for i in range(len(cv_image_h_histogram_cut))
        if cv_image_h_histogram_cut[(i - 1) % histogram_length] < cv_image_h_histogram_cut[i] > cv_image_h_histogram_cut[(i + 1) % histogram_length]]
    histogram_high_peaks = filter(lambda x : cv_image_h_histogram_cut[x] > histogram_peak_cut, histogram_peaks)
    #print(histogram_high_peaks)
    #pyplot.plot(cv_image_h_histogram_cut)
    #pyplot.show()

    # Process every color found in the histogram
    blob_info = {}
    cv_image_contours_debug = cv2.cvtColor(cv_image_binary, cv2.COLOR_GRAY2BGR)
    for current_hue in histogram_high_peaks:
        # Perform a Hue rotation that will be used to make detecting the edge colors easier (red in HSV corresponds to both 0 and 180)
        HUE_AMPLITUDE = 5
        cv_image_h_rotated = cv_image_h.copy()
        cv_image_h_rotated[:] -= current_hue
        cv_image_h_rotated[:] += HUE_AMPLITUDE

        # Binarize using range function
        cv_image_h_inrange = cv2.inRange(cv_image_h_rotated, 0, HUE_AMPLITUDE * 2)

        # Apply binary mask (consider that both black and the edge color have hue 0)
        cv_image_h_masked = cv2.bitwise_and(cv_image_h_inrange, cv_image_h_inrange, mask=cv_image_binary)

        # Erode
        EROSION_SIZE = 5
        erosion_kernel = numpy.ones((EROSION_SIZE, EROSION_SIZE), numpy.uint8)
        cv_image_h_eroded = cv2.erode(cv_image_h_masked, erosion_kernel)
        #cv2.imshow("inRange {}".format(histogram_high_peaks.index(current_hue)), cv_image_h_eroded)

        # Find convex contours
        _, contours, _ = cv2.findContours(cv_image_h_eroded.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        convex_contours = [cv2.convexHull(cnt) for cnt in contours]

        contour_color_hsv = numpy.array([[[current_hue, 255, 255]]], numpy.uint8)
        contour_color_rgb = cv2.cvtColor(contour_color_hsv, cv2.COLOR_HSV2BGR)[0][0].tolist()
        #cv2.drawContours(cv_image_contours_debug, convex_contours, -1, contour_color_rgb, 1)

        # Find centroids
        contour_moments = [cv2.moments(cnt) for cnt in convex_contours]
        contour_centroids = [(int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"])) for moments in contour_moments if moments["m00"] != 0]
        for (cx, cy) in contour_centroids:
            cv2.circle(cv_image_contours_debug, (cx, cy), 3, contour_color_rgb, -1)

        # Collect data
        blob_info[current_hue] = contour_centroids

    cv2.imshow("Convex contours", cv_image_contours_debug)

    return blob_info

def __publish_marker(publisher, index, point):
    """
    Publishes a cube-shaped marker

    :param index: The marker index
    :param point: The 3D position of the marker
    """
    marker = Marker()
    marker.header.frame_id = "base"
    marker.id = index
    marker.type = Marker.CUBE
    marker.scale.x = 0.04
    marker.scale.y = 0.04
    marker.scale.z = 0.04
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = point[2]

    publisher.publish(marker)

def test_ros():
    """
    Test the blob detection and CameraHelper class using ROS
    """
    rospy.init_node('cv_detection')

    camera_name = "head_camera"

    TABLE_HEIGHT = -0.12
    camera_helper = CameraHelper(camera_name, "base", TABLE_HEIGHT)

    bridge = CvBridge()

    publisher = rospy.Publisher("cube_position_estimation", Marker, queue_size=10)

    try:
        while not rospy.is_shutdown():
            # Take picture
            img_data = camera_helper.take_single_picture()

            # Convert to OpenCV format
            cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")

            # Save for debugging
            #cv2.imwrite("debug.png", cv_image)

            # Get color blobs info
            blob_info = get_blob_info(cv_image)

            # Project the points on 3D space
            points = [y for x in blob_info.values() for y in x]

            for index, point in enumerate(points):
                projected = camera_helper.project_point_on_table(point)
                __publish_marker(publisher, index, projected)

            # Wait for a key press
            cv2.waitKey(1)

            rospy.sleep(0.1)
    except CvBridgeError, err:
        rospy.logerr(err)

    # Exit
    cv2.destroyAllWindows()

def test_debug():
    """
    Test the blob detection using images on disk
    """

    # Get files
    path = rospkg.RosPack().get_path('sorting_demo') + "/share"
    files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f)) and "head-mask" not in f]
    #print(files)

    # Process files
    for f in files:
        # Get image path
        image_path = os.path.join(path, f)
        print(image_path)

        # Read image
        cv_image = cv2.imread(image_path)

        # Get color blobs info
        blob_info = get_blob_info(cv_image)
        print(blob_info)

        # Wait for a key press
        cv2.waitKey(0)

    # Exit
    cv2.destroyAllWindows()

if __name__ == '__main__':
    sys.exit(test_debug())
