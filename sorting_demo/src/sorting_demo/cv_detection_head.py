#!/usr/bin/python
import sys
import numpy
import os

from matplotlib import pyplot

import rospy
import cv2
import rospkg

from cv_bridge import CvBridge, CvBridgeError

from visualization_msgs.msg import *

from cv_detection_camera_helper import CameraHelper

import demo_constants

import intera_interface

use_real_robot_parameters = demo_constants.is_real_robot()

class HeadCVParameters:
    def __init__(self):
        global use_real_robot_parameters
        if use_real_robot_parameters:
            self.BLUR_SIZE = 3
            self.ROI_MASK = rospkg.RosPack().get_path('sorting_demo') + "/share/head_mask_real.png"
            self.CLAHE_SIZE = 16
            self.BIN_THRESHOLD = 16
            self.EROSION_SIZE = 3
            self.HISTOGRAM_BLUR_SIZE = 61
            self.PEAK_MINIMUM = 10
            self.PEAK_MAXIMUM = 150
            self.HUE_AMPLITUDE = 10
            self.FIND_CONTOURS_METHOD = cv2.RETR_EXTERNAL
            self.MINIMUM_AREA_SIZE = 50

        else: # Simulator
            self.BLUR_SIZE = 3
            self.ROI_MASK = rospkg.RosPack().get_path('sorting_demo') + "/share/head_mask_simulator.png"
            self.CLAHE_SIZE = 16
            self.BIN_THRESHOLD = 64
            self.EROSION_SIZE = 3
            self.HISTOGRAM_BLUR_SIZE = 11
            self.PEAK_MINIMUM = 100
            self.PEAK_MAXIMUM = 500
            self.HUE_AMPLITUDE = 5
            self.FIND_CONTOURS_METHOD = cv2.RETR_LIST
            self.MINIMUM_AREA_SIZE = 50

def get_blobs_info(cv_image):
    """
    Gets information about the colored blobs in the image

    :param cv_image: The OpenCV image to get blobs from
    :return: A dictionary containing an array of points for each colored blob found, represented by its hue.
        For example, in an image with 2 red blobs and 3 blue blobs, it will return
            {0: [(639, 558), (702, 555)], 120: [(567, 550), (698, 515), (648, 515)]}
    """

    head_parameters = HeadCVParameters()

    # Show original image
    #cv2.imshow("Original image", cv_image)

    # Apply blur
    BLUR_SIZE = head_parameters.BLUR_SIZE
    cv_image_blur = cv2.GaussianBlur(cv_image, (BLUR_SIZE, BLUR_SIZE), 0)
    #cv2.imshow("Blur", cv_image_blur)

    # Apply ROI mask
    ROI_MASK = head_parameters.ROI_MASK
    cv_mask = cv2.imread(ROI_MASK)
    cv_mask = cv2.cvtColor(cv_mask, cv2.COLOR_BGR2GRAY)

    cv_image_masked = cv2.bitwise_and(cv_image_blur, cv_image_blur, mask=cv_mask)
    #cv2.imshow("Masked original", cv_image_masked)

    # HSV split
    cv_image_hsv = cv2.cvtColor(cv_image_masked, cv2.COLOR_BGR2HSV)
    cv_image_h, cv_image_s, cv_image_v = cv2.split(cv_image_hsv)
    #cv2.imshow("Image H", cv_image_h)
    #cv2.imshow("Image S", cv_image_s)
    #cv2.imshow("Image V", cv_image_v)

    # Apply CLAHE to Value channel
    CLAHE_SIZE = head_parameters.CLAHE_SIZE
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(CLAHE_SIZE, CLAHE_SIZE))
    cv_image_v_clahe = clahe.apply(cv_image_v)

    # Merge channels
    cv_image_clahe = cv2.merge((cv_image_h, cv_image_s, cv_image_v_clahe))
    #cv_image_clahe_rgb = cv2.cvtColor(cv_image_clahe, cv2.COLOR_HSV2BGR)
    #cv2.imshow("CLAHE", cv_image_clahe_rgb)

    # Multiply Saturation and Value channels to separate the cubes, removing the table
    cv_image_sv_multiplied = cv2.multiply(cv_image_s, cv_image_v_clahe, scale=1/255.0)
    #cv2.imshow("Image S*V", cv_image_sv_multiplied)

    # Binarize the result
    BIN_THRESHOLD = head_parameters.BIN_THRESHOLD
    #_, cv_image_binary = cv2.threshold(cv_image_sv_multiplied, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    _, cv_image_binary = cv2.threshold(cv_image_sv_multiplied, BIN_THRESHOLD, 255, cv2.THRESH_BINARY)
    #cv2.imshow("Threshold", cv_image_binary)

    # Erode the image to use as mask
    EROSION_SIZE = head_parameters.EROSION_SIZE
    erosion_kernel = numpy.ones((EROSION_SIZE, EROSION_SIZE), numpy.uint8)
    cv_image_binary_eroded = cv2.erode(cv_image_binary, erosion_kernel)

    # Calculate H-channel histogram, applying the binarized image as mask
    cv_image_h_histogram = cv2.calcHist([cv_image_h], [0], cv_image_binary_eroded, [180], [0, 179])

    # Smoothen the histogram to find local maxima
    HISTOGRAM_BLUR_SIZE = head_parameters.HISTOGRAM_BLUR_SIZE
    histogram_length = len(cv_image_h_histogram)
    cv_image_h_histogram_wrap = cv2.repeat(cv_image_h_histogram, 3, 1)
    cv_image_h_histogram_smooth = cv2.GaussianBlur(cv_image_h_histogram_wrap, (HISTOGRAM_BLUR_SIZE, HISTOGRAM_BLUR_SIZE), 0)
    cv_image_h_histogram_cut = cv_image_h_histogram_smooth[histogram_length : 2 * histogram_length]

    # Collect peaks
    histogram_peaks = [i for i in range(len(cv_image_h_histogram_cut))
        if cv_image_h_histogram_cut[(i - 1) % histogram_length] < cv_image_h_histogram_cut[i] > cv_image_h_histogram_cut[(i + 1) % histogram_length]]

    # Filter peaks
    PEAK_MINIMUM = head_parameters.PEAK_MINIMUM # Ideally below the value of a single cube
    PEAK_MAXIMUM = head_parameters.PEAK_MAXIMUM # Ideally above the value of all the cubes of a single color
    histogram_high_peaks = filter(lambda p : PEAK_MINIMUM < cv_image_h_histogram_cut[p] < PEAK_MAXIMUM, histogram_peaks)
    #print(histogram_high_peaks)
    #pyplot.clf()
    #pyplot.plot(cv_image_h_histogram_cut)
    #pyplot.pause(0.001)
    #pyplot.show()

    # Process every color found in the histogram
    blob_info = {}
    cv_image_contours_debug = cv2.cvtColor(cv_image_binary_eroded, cv2.COLOR_GRAY2BGR)

    for current_hue in histogram_high_peaks:
        # Perform a Hue rotation that will be used to make detecting the edge colors easier (red in HSV corresponds to both 0 and 180)
        HUE_AMPLITUDE = head_parameters.HUE_AMPLITUDE
        cv_image_h_rotated = cv_image_h.astype(numpy.int16)
        cv_image_h_rotated[:] -= current_hue
        cv_image_h_rotated[:] += HUE_AMPLITUDE
        cv_image_h_rotated = numpy.remainder(cv_image_h_rotated, 180)
        cv_image_h_rotated = cv_image_h_rotated.astype(numpy.uint8)
        #cv2.imshow("Hue rotation {}".format(histogram_high_peaks.index(current_hue)), cv_image_h_rotated)

        # Binarize using range function
        cv_image_h_inrange = cv2.inRange(cv_image_h_rotated, 0, HUE_AMPLITUDE * 2)

        # Apply binary mask (consider that both black and the edge color have hue 0)
        cv_image_h_masked = cv2.bitwise_and(cv_image_h_inrange, cv_image_h_inrange, mask=cv_image_binary_eroded)
        #cv2.imshow("inRange {}".format(histogram_high_peaks.index(current_hue)), cv_image_h_masked)

        # Find contours
        FIND_CONTOURS_METHOD = head_parameters.FIND_CONTOURS_METHOD
        _, contours, _ = cv2.findContours(cv_image_h_masked.copy(), FIND_CONTOURS_METHOD, cv2.CHAIN_APPROX_SIMPLE)

        # Filter by area
        MINIMUM_AREA_SIZE = head_parameters.MINIMUM_AREA_SIZE
        contours_filtered_area = filter(lambda cnt: cv2.contourArea(cnt) > MINIMUM_AREA_SIZE, contours)

        # Calculate convex hull
        convex_contours = [cv2.convexHull(cnt) for cnt in contours_filtered_area]

        contour_color_hsv = numpy.array([[[current_hue, 255, 255]]], numpy.uint8)
        contour_color_rgb = cv2.cvtColor(contour_color_hsv, cv2.COLOR_HSV2BGR)[0][0].tolist()
        cv2.drawContours(cv_image_contours_debug, convex_contours, -1, contour_color_rgb, 1)

        # Find centroids
        contour_moments = [cv2.moments(cnt) for cnt in convex_contours]
        contour_centroids = [(int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"])) for moments in contour_moments if moments["m00"] != 0]
        for (cx, cy) in contour_centroids:
            cv2.circle(cv_image_contours_debug, (cx, cy), 3, contour_color_rgb, cv2.FILLED)

        # Collect data
        blob_info[current_hue] = contour_centroids

    #cv2.imshow("Blobs info", cv_image_contours_debug)

    image_file_name = "/tmp/head_contours.jpg"
    cv2.imwrite(image_file_name, cv_image_contours_debug)

    # Create head display image
    head_display_image = __create_head_screen_image(cv_mask, cv_image_contours_debug)

    head_display_image_file_name = "/tmp/debug_head.png"
    cv2.imwrite(head_display_image_file_name, head_display_image)

    # Show image on head display
    head_display = intera_interface.HeadDisplay()
    head_display.display_image(head_display_image_file_name)

    return blob_info

def __create_head_screen_image(cv_mask, cv_image_contours_debug):
    # Compute bounding rectangle for the head display
    _, contours, _ = cv2.findContours(cv_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x, y, w, h = cv2.boundingRect(contours[0])

    # Get ROI from the debug image
    cv_image_contours_debug_roi = cv_image_contours_debug[y:y+h,x:x+w]

    # Scale image
    target_height, target_width = (600, 1024)
    scale = min(target_width / float(w), target_height / float(h))

    new_width = int(scale * w)
    new_height = int(scale * h)
    cv_image_contours_debug_resized = cv2.resize(cv_image_contours_debug_roi, (new_width, new_height), interpolation = cv2.INTER_CUBIC)

    # Put the image in black background
    cv_image_head = numpy.zeros((target_height, target_width, 3), numpy.uint8)
    cv_image_head[:new_height,:new_width] = cv_image_contours_debug_resized

    return cv_image_head

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

def test_head_ros():
    """
    Test the blob detection and CameraHelper class using ROS
    """
    rospy.init_node('cv_detection_head_camera')

    camera_name = "head_camera"

    TABLE_HEIGHT = demo_constants.TABLE_HEIGHT_FOR_PROJECTION
    camera_helper = CameraHelper(camera_name, "base", TABLE_HEIGHT)
    camera_helper.set_exposure(100)
    camera_helper.set_gain(30)

    bridge = CvBridge()

    publisher = rospy.Publisher("cube_position_estimation", Marker, queue_size=10)

    try:
        while not rospy.is_shutdown():
            # Take picture
            img_data = camera_helper.take_single_picture()

            # Convert to OpenCV format
            cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")

            # Save for debugging
            #cv2.imwrite("/tmp/debug.png", cv_image)

            # Get color blobs info
            blob_info = get_blobs_info(cv_image)

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

def test_head_cam():
    """
    Test the blob detection using a USB camera
    """

    # Create a video capture object
    capture = cv2.VideoCapture(1)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    while True:
        # Capture a frame
        ret, cv_image = capture.read()

        if ret:
            # Save for debugging
            #cv2.imwrite("/tmp/debug.png", cv_image)

            # Get color blobs info
            blob_info = get_blobs_info(cv_image)
            print(blob_info)

            # Check for a press on the Escape key
            if cv2.waitKey(1) & 0xFF == 27:
                break

    # Exit
    capture.release()
    cv2.destroyAllWindows()

def test_head_debug():
    """
    Test the blob detection using images on disk
    """

    # Get files
    global use_real_robot_parameters
    if use_real_robot_parameters:
        path = rospkg.RosPack().get_path('sorting_demo') + "/share/test_head_real"
    else:
        path = rospkg.RosPack().get_path('sorting_demo') + "/share/test_head_simulator"

    files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
    #print(files)

    # Process files
    for f in files:
        # Get image path
        image_path = os.path.join(path, f)
        print(image_path)

        # Read image
        cv_image = cv2.imread(image_path)

        # Get color blobs info
        blob_info = get_blobs_info(cv_image)
        print(blob_info)

        # Wait for a key press
        cv2.waitKey(0)

    # Exit
    cv2.destroyAllWindows()

if __name__ == '__main__':
    sys.exit(test_head_debug())
