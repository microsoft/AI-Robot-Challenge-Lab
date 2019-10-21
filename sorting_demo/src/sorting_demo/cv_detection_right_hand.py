#!/usr/bin/python
import sys
import numpy
import os
import math
import functools
import time

import rospy
import cv2
import rospkg

from cv_bridge import CvBridge, CvBridgeError

from cv_detection_camera_helper import CameraHelper

import demo_constants

import intera_interface

use_real_robot_parameters = demo_constants.is_real_robot() # or True

class RightHandCVParameters:
    def __init__(self):
        global use_real_robot_parameters
        if use_real_robot_parameters:
            self.BLUR_SIZE = 3



            self.CUBE_SIZE = 120
            self.CUBE_BORDER_SIZE = 4
            self.CLEARANCE_AREA_LENGTH = self.CUBE_SIZE / 2
            self.CLEARANCE_AREA_MARGIN = 20
            self.TEMPLATE_MATCHING_MAX_THRESHOLD = 0.5
            self.CLEARANCE_THRESHOLD = 1000

        else: # Simulator
            self.BLUR_SIZE = 3
            self.CLAHE_SIZE = 64
            self.SIGMA = 0.33
            self.DILATION_SIZE = 5
            self.CUBE_SIZE = 110
            self.CUBE_BORDER_SIZE = 4
            self.CLEARANCE_AREA_LENGTH = self.CUBE_SIZE / 2
            self.CLEARANCE_AREA_MARGIN = 20
            self.TEMPLATE_MATCHING_MAX_THRESHOLD = 0.5
            self.CLEARANCE_THRESHOLD = 50

imgindex = 0

def __create_mask_image_from_template(reference_image, template, pos_x, pos_y):
    '''
    Resize a template image to match a reference image size, placing the template's center in the given position, to be used as a mask

    :param reference_image: The reference image that the returned mask is intended to be applied to. The returned mask will have the same size as this image
    :param template: The template to resize
    :param pos_x: The x position where to place the center of the template in the final image
    :param pos_y: The y position where to place the center of the template in the final image
    :return: The resized, correctly positioned template
    '''

    # Get image and template sizes
    reference_image_height, reference_image_width = reference_image.shape
    template_height, template_width = template.shape

    # Get the position the template should be placed at
    pos_x_corrected = pos_x - template_width // 2
    pos_y_corrected = pos_y - template_height // 2

    # Calculate bottom and right margins
    bottom_margin = reference_image_height - template_height - pos_y_corrected
    right_margin = reference_image_width - template_width - pos_x_corrected

    # Add the borders to the template image
    border_top = max(0, pos_y_corrected)
    border_bottom = max(0, bottom_margin)
    border_left = max(0, pos_x_corrected)
    border_right = max(0, right_margin)
    mask_image = cv2.copyMakeBorder(template, border_top, border_bottom, border_left, border_right, cv2.BORDER_CONSTANT, value=0)

    # Crop the image, in case the template ended up outside of the image
    crop_top = int(math.fabs(min(0, pos_y_corrected)))
    crop_bottom = crop_top + reference_image_height
    crop_left = int(math.fabs(min(0, pos_x_corrected)))
    crop_right = crop_left + reference_image_width
    mask_image_cropped = mask_image[crop_top:crop_bottom,crop_left:crop_right]

    return mask_image_cropped

def __rotate_image_size_corrected(image, angle):
    # Calculate max size for the rotated template and image offset
    image_size_height, image_size_width = image.shape
    image_center_x = image_size_width // 2
    image_center_y = image_size_height // 2

    # Create rotation matrix
    rotation_matrix = cv2.getRotationMatrix2D((image_center_x, image_center_y), -angle, 1)

    # Apply offset
    new_image_size = int(math.ceil(cv2.norm((image_size_height, image_size_width), normType=cv2.NORM_L2)))
    rotation_matrix[0, 2] += (new_image_size - image_size_width) / 2
    rotation_matrix[1, 2] += (new_image_size - image_size_height) / 2

    # Apply rotation to the template
    image_rotated = cv2.warpAffine(image, rotation_matrix, (new_image_size, new_image_size))
    return image_rotated

def __apply_template_matching(angle, template, image):
    # Rotate the template
    template_rotated = __rotate_image_size_corrected(template, angle)

    # Apply template matching
    image_templated = cv2.matchTemplate(image, template_rotated, cv2.TM_CCOEFF_NORMED)

    # Correct template matching image size difference
    template_rotated_height, template_rotated_width = template_rotated.shape
    template_half_height = template_rotated_height // 2
    template_half_width = template_rotated_width // 2

    image_templated_inrange_size_corrected = cv2.copyMakeBorder(image_templated, template_half_height, template_half_height, template_half_width, template_half_width, cv2.BORDER_CONSTANT, value=0)

    # Calculate maximum match coefficient
    max_match = numpy.max(image_templated_inrange_size_corrected)

    return (max_match, angle, template_rotated, image_templated_inrange_size_corrected)

def get_cubes_z_rotation(cv_image):
    """
    Gets the cubes rotation in the Z plane from an image. The results are sorted by distance to the center of the image

    :param cv_image: The OpenCV image to get the cubes from
    :return: An array containing the positions, angles and clearances of the cubes that have been found. The returned angles lie in the interval [-45, 45)
    For example:
        [((388, 526), -41.0, True, True), ((556, 524), -31.0, True, True), ((474, 382), -31.0, True, False)]

    """

    right_hand_parameters = RightHandCVParameters()

    # Show original image
    #cv2.imshow("Original image", cv_image)

    # Convert to grayscale
    cv_image_grayscale = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Apply blur
    BLUR_SIZE = right_hand_parameters.BLUR_SIZE
    cv_image_blur = cv2.GaussianBlur(cv_image_grayscale, (BLUR_SIZE, BLUR_SIZE), 0)
    #cv2.imshow("Blur", cv_image_blur)

    global use_real_robot_parameters
    if use_real_robot_parameters:
        # Apply adaptive threshold
        _, cv_image_dilated = cv2.threshold(cv_image_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        #cv2.imshow("Threshold", cv_image_dilated)
    else:
        # Apply CLAHE
        CLAHE_SIZE = right_hand_parameters.CLAHE_SIZE
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(CLAHE_SIZE, CLAHE_SIZE))
        cv_image_clahe = clahe.apply(cv_image_blur)
        #cv2.imshow("CLAHE", cv_image_clahe)

        # Apply Canny filter
        SIGMA = right_hand_parameters.SIGMA
        median = numpy.median(cv_image_clahe)
        lower = int(max(0, (1.0 - SIGMA) * median))
        upper = int(min(255, (1.0 + SIGMA) * median))
        cv_image_canny = cv2.Canny(cv_image_clahe, lower, upper)
        #cv2.imshow("Canny", cv_image_canny)

        # Apply dilation
        DILATION_SIZE = right_hand_parameters.DILATION_SIZE
        dilation_kernel = numpy.ones((DILATION_SIZE, DILATION_SIZE), numpy.uint8)
        cv_image_dilated = cv2.dilate(cv_image_canny, dilation_kernel, iterations = 1)
        #cv2.imshow("Dilation", cv_image_dilated)

    # Find contours
    _, contours, _ = cv2.findContours(cv_image_dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Simplify contours
    #contours_approx = [cv2.approxPolyDP(cnt, 0.005 * cv2.arcLength(cnt, True), True) for cnt in contours]

    # Draw image with filled contours
    cv_image_height, cv_image_width = cv_image_grayscale.shape
    cv_image_contours_filled = numpy.zeros((cv_image_height, cv_image_width), numpy.uint8)
    cv_image_contours_filled = cv2.fillPoly(cv_image_contours_filled, pts=contours, color=255)
    #cv2.imshow("Contours", cv_image_contours_filled)

    # Create cube image for template matching
    CUBE_SIZE = right_hand_parameters.CUBE_SIZE
    cv_image_cube_template = numpy.full((CUBE_SIZE, CUBE_SIZE, 1), 255, numpy.uint8)

    CUBE_BORDER_SIZE = right_hand_parameters.CUBE_BORDER_SIZE
    cv_image_cube_template_border = cv2.copyMakeBorder(cv_image_cube_template, CUBE_BORDER_SIZE, CUBE_BORDER_SIZE, CUBE_BORDER_SIZE, CUBE_BORDER_SIZE, cv2.BORDER_CONSTANT, value=0)

    # Create mask for clearance check
    CLEARANCE_AREA_LENGTH = right_hand_parameters.CLEARANCE_AREA_LENGTH
    CLEARANCE_AREA_MARGIN = right_hand_parameters.CLEARANCE_AREA_MARGIN
    clearance_check_mask = numpy.full((CUBE_SIZE + 2 * CLEARANCE_AREA_MARGIN, CUBE_SIZE), 0, numpy.uint8)
    clearance_check_mask = cv2.copyMakeBorder(clearance_check_mask, CLEARANCE_AREA_LENGTH, CLEARANCE_AREA_LENGTH, 0, 0, cv2.BORDER_CONSTANT, value=255)

    # Calculate the angles that will be used when rotating the template (between -45 and 45 because of square symmetry)
    range_subdivisions = 90
    all_angles = numpy.arange(-45, 45, 90.0 / range_subdivisions)

    # Look for cubes in the image and erase them until none are found
    cube_positions_and_angles = []
    original_image_loop = cv_image_contours_filled.copy()
    while True:
        #cv2.imshow("Loop image", original_image_loop)
        #cv2.waitKey(0)

        # Apply template matching rotating the template
        apply_template_matching_partial = functools.partial(__apply_template_matching, template=cv_image_cube_template_border, image=original_image_loop)
        template_matching_results = map(apply_template_matching_partial, all_angles)

        # Get max matching coefficient
        template_matching_results_max_values = [value for value, _, _, _ in template_matching_results]
        template_matching_results_max = max(template_matching_results_max_values)

        # Check if the match coefficient is good enough
        TEMPLATE_MATCHING_MAX_THRESHOLD = right_hand_parameters.TEMPLATE_MATCHING_MAX_THRESHOLD
        if template_matching_results_max < TEMPLATE_MATCHING_MAX_THRESHOLD:
            break

        # Collect best match
        template_matching_results_max_index = template_matching_results_max_values.index(template_matching_results_max)
        _, angle, template_rotated, image_templated = template_matching_results[template_matching_results_max_index]

        # Find location
        _, _, _, (max_loc_x, max_loc_y) = cv2.minMaxLoc(image_templated)

        # Apply template as a mask to the original image, deleting the area it matched
        template_mask = __create_mask_image_from_template(original_image_loop, template_rotated, max_loc_x, max_loc_y)

        template_mask_inverted = cv2.bitwise_not(template_mask)

        original_image_loop = cv2.bitwise_and(original_image_loop, template_mask_inverted)

        # Rotate the clearance check mask to create the two needed for the clearance test
        clearance_check_mask_rotated_0 = __rotate_image_size_corrected(clearance_check_mask, angle)
        clearance_check_mask_rotated_90 = __rotate_image_size_corrected(clearance_check_mask, angle + 90)
        #cv2.imshow("Clearance check mask rotated 0", clearance_check_mask_rotated_0)
        #cv2.imshow("Clearance check mask rotated 90", clearance_check_mask_rotated_90)

        # Create mask image from the clearance check mask
        clearance_check_mask_full_size_0 = __create_mask_image_from_template(cv_image_contours_filled, clearance_check_mask_rotated_0, max_loc_x, max_loc_y)
        clearance_check_mask_full_size_90 = __create_mask_image_from_template(cv_image_contours_filled, clearance_check_mask_rotated_90, max_loc_x, max_loc_y)
        #cv2.imshow("Clearance check mask 0 full", clearance_check_mask_full_size_0)
        #cv2.imshow("Clearance check mask 90 full", clearance_check_mask_full_size_90)

        # Apply clearance mask to the original filled-contours image
        original_image = cv_image_contours_filled
        original_image_clearance_mask_applied_0 = cv2.bitwise_and(original_image, clearance_check_mask_full_size_0)
        original_image_clearance_mask_applied_90 = cv2.bitwise_and(original_image, clearance_check_mask_full_size_90)
        #cv2.imshow("Clearance check mask 0 applied", original_image_clearance_mask_applied_0)
        #cv2.imshow("Clearance check mask 90 applied", original_image_clearance_mask_applied_90)
        #cv2.waitKey(0)

        # Check clearance
        clearance_0_count = cv2.countNonZero(original_image_clearance_mask_applied_0)
        clearance_90_count = cv2.countNonZero(original_image_clearance_mask_applied_90)
        #print(clearance_0_count)
        #print(clearance_90_count)

        CLEARANCE_THRESHOLD = right_hand_parameters.CLEARANCE_THRESHOLD
        clearance_0 = clearance_0_count < CLEARANCE_THRESHOLD
        clearance_90 = clearance_90_count < CLEARANCE_THRESHOLD

        # Store result
        cube_positions_and_angles.append(((max_loc_x, max_loc_y), angle, clearance_0, clearance_90))

    # Sort cube positions by distance to the center of the image
    image_center_x = cv_image_height // 2
    image_center_y = cv_image_width // 2
    image_center = (image_center_x, image_center_y)
    cube_positions_and_angles_sorted = sorted(cube_positions_and_angles, key=lambda (position, angle, clearance_0, clearance_90) : cv2.norm(position, image_center, normType=cv2.NORM_L2))

    # Draw debug image
    template_matching_debug_image = cv_image.copy()
    for ((x, y), angle, clear_0, clear_90) in cube_positions_and_angles_sorted:
        rotated_rectangle = cv2.boxPoints(((x, y), (CUBE_SIZE, CUBE_SIZE), angle))
        rotated_rectangle = numpy.int0(rotated_rectangle)
        cv2.drawContours(template_matching_debug_image, [rotated_rectangle], -1, (255, 0, 0), 4)

        clearance_points_rectangle = cv2.boxPoints(((x, y), (CUBE_SIZE * 0.5, CUBE_SIZE * 0.5), angle + 45))
        clearance_points_rectangle = numpy.int0(clearance_points_rectangle)

        clearance_bools = [clear_90, clear_0]
        for (i, (point_x, point_y)) in enumerate(clearance_points_rectangle):
            current_clearance = clearance_bools[i % len(clearance_bools)]
            clearance_circle_color = current_clearance and (0, 255, 0) or (0, 0, 255)
            cv2.circle(template_matching_debug_image, (point_x, point_y), 8, clearance_circle_color, cv2.FILLED)
            cv2.circle(template_matching_debug_image, (point_x, point_y), 8, (255, 255, 255), 3)

    #cv2.imshow("Template matching result", template_matching_debug_image)
    global imgindex
    image_file_name = "/tmp/img"+ str(imgindex)+".jpg"
    cv2.imwrite(image_file_name, template_matching_debug_image)
    imgindex+=1
    #cv2.waitKey(0)

    # Resize image for head display
    debug_resized = cv2.resize(template_matching_debug_image, (1024, 600))

    image_file_name_resized = "/tmp/debug_right_hand_resized.png"
    cv2.imwrite(image_file_name_resized, debug_resized)

    # Show image on head display
    head_display = intera_interface.HeadDisplay()
    head_display.display_image(image_file_name_resized)

    return cube_positions_and_angles_sorted

def test_right_hand_ros():
    """
    Test the cube orientation sensing using ROS
    """
    rospy.init_node('cv_detection_right_hand_camera')

    camera_name = "right_hand_camera"

    camera_helper = CameraHelper(camera_name, "base")

    if demo_constants.is_real_robot():
        camera_helper.set_exposure(5)
        camera_helper.set_gain(255)

    bridge = CvBridge()

    index = 0
    try:
        while not rospy.is_shutdown():
            # Take picture
            camera_helper.set_cognex_strobe(True)
            time.sleep(0.05)
            img_data = camera_helper.take_single_picture()
            camera_helper.set_cognex_strobe(False)

            # Convert to OpenCV format
            cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")

            # Save for debugging
            cv2.imwrite("/tmp/right_hand_debug{}.png".format(index), cv_image)
            index += 1

            # Get cube rotation
            angles = get_cubes_z_rotation(cv_image)
            print(angles)

            # Wait for a key press
            cv2.waitKey(1)

            rospy.sleep(1)
    except CvBridgeError, err:
        rospy.logerr(err)

    # Exit
    cv2.destroyAllWindows()

def test_right_hand_cam():
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

            # Get cube rotation
            angles = get_cubes_z_rotation(cv_image)
            print(angles)

            # Check for a press on the Escape key
            if cv2.waitKey(1) & 0xFF == 27:
                break

    # Exit
    capture.release()
    cv2.destroyAllWindows()

def test_right_hand_debug():
    """
    Test the cube orientation sensing using images on disk
    """

    # Get files
    global use_real_robot_parameters
    if use_real_robot_parameters:
        path = rospkg.RosPack().get_path('sorting_demo') + "/share/test_right_hand_real"
    else:
        path = rospkg.RosPack().get_path('sorting_demo') + "/share/test_right_hand_simulator"

    files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
    #files = ["border.png"]
    #print(files)

    # Process files
    for f in files:
        # Get image path
        image_path = os.path.join(path, f)
        print(image_path)

        # Read image
        cv_image = cv2.imread(image_path)

        # Get cube rotation
        angles = get_cubes_z_rotation(cv_image)
        print(angles)

        # Wait for a key press
        cv2.waitKey(0)

    # Exit
    cv2.destroyAllWindows()

if __name__ == '__main__':
    sys.exit(test_right_hand_debug())
