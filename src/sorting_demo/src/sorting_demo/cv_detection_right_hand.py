#!/usr/bin/python
import sys
import numpy
import os
import functools

import rospy
import cv2
import rospkg

from cv_bridge import CvBridge, CvBridgeError

from cv_detection_camera_helper import CameraHelper

def __apply_template_matching(angle, template, image):
    # Calculate max size for the rotated template and image offset
    template_size, _, _ = template.shape
    new_template_size = int(template_size * 2 * 0.7072)
    offset = (new_template_size - template_size) / 2

    # Create rotation matrix
    rotation_matrix = cv2.getRotationMatrix2D((template_size / 2, template_size / 2), angle, 1)

    # Apply offset
    rotation_matrix[0, 2] += offset
    rotation_matrix[1, 2] += offset

    # Apply rotation to the template
    template_rotated = cv2.warpAffine(template, rotation_matrix, (new_template_size, new_template_size))

    # Apply template matching
    image_templated = cv2.matchTemplate(image, template_rotated, cv2.TM_CCOEFF_NORMED)

    # Correct template matching image size difference
    template_rotated_height, template_rotated_width = template_rotated.shape
    template_half_height = template_rotated_height / 2
    template_half_width = template_rotated_width / 2

    image_templated_inrange_size_corrected = cv2.copyMakeBorder(image_templated, template_half_height, template_half_height, template_half_width, template_half_width, cv2.BORDER_CONSTANT, 0)

    # Calculate maximum match coefficient
    max_match = numpy.max(image_templated_inrange_size_corrected)

    return (max_match, -angle, template_rotated, image_templated_inrange_size_corrected)

def get_cubes_z_rotation(cv_image):
        """
        Gets the cubes rotation in the Z plane from an image.

        :param cv_image: The OpenCV image to get the cube rotation from
        :return: An array containing the positions and angles of the cubes that have been found.
        For example:
            [((405, 308), -7.0), ((393, 211), -7.0), ((506, 202), -56.0)]
        """

        # Show original image
        #cv2.imshow("Original image", cv_image)

        # Convert to grayscale
        cv_image_grayscale = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Apply blur
        BLUR_SIZE = 3
        cv_image_blur = cv2.GaussianBlur(cv_image_grayscale, (BLUR_SIZE, BLUR_SIZE), 0)
        #cv2.imshow("Blur", cv_image_blur)

        # Apply CLAHE
        CLAHE_SIZE = 64
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(CLAHE_SIZE, CLAHE_SIZE))
        cv_image_clahe = clahe.apply(cv_image_blur)
        #cv2.imshow("CLAHE", cv_image_clahe)

        # Apply Canny filter
        cv_image_canny = cv2.Canny(cv_image_clahe, 255/3, 255)
        #cv2.imshow("Canny", cv_image_canny)

        # Apply dilation
        DILATION_SIZE = 5
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
        CUBE_SIZE = 98
        cv_image_cube_template = numpy.full((CUBE_SIZE, CUBE_SIZE, 1), 255, numpy.uint8)

        CUBE_BORDER_SIZE = 4
        cv_image_cube_template_border = numpy.zeros((CUBE_SIZE + 2 * CUBE_BORDER_SIZE, CUBE_SIZE + 2 * CUBE_BORDER_SIZE, 1), numpy.uint8)
        cv_image_cube_template_border[CUBE_BORDER_SIZE:-CUBE_BORDER_SIZE, CUBE_BORDER_SIZE:-CUBE_BORDER_SIZE] = cv_image_cube_template

        # Calculate the angles that will be used when rotating the template (between 0 and 90 because of square symmetry)
        range_subdivisions = 90
        all_angles = numpy.arange(0, 90, 90.0 / range_subdivisions)

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
            TEMPLATE_MATCHING_MAX_THRESHOLD = 0.7
            if template_matching_results_max < TEMPLATE_MATCHING_MAX_THRESHOLD:
                break

            # Collect best match
            template_matching_results_max_index = template_matching_results_max_values.index(template_matching_results_max)
            _, angle, template_rotated, image_templated = template_matching_results[template_matching_results_max_index]

            # Find location
            _, _, _, (max_loc_x, max_loc_y) = cv2.minMaxLoc(image_templated)

            # Store result
            cube_positions_and_angles.append(((max_loc_x, max_loc_y), angle))

            # Apply template as a mask to the original image, deleting the area it matched
            template_rotated_height, template_rotated_width = template_rotated.shape
            max_loc_x_corrected = max_loc_x - template_rotated_width / 2
            max_loc_y_corrected = max_loc_y - template_rotated_height / 2

            original_image_loop_height, original_image_loop_width = original_image_loop.shape
            bottom_margin = original_image_loop_height - template_rotated_height - max_loc_y_corrected
            right_margin = original_image_loop_width - template_rotated_width - max_loc_x_corrected
            template_mask = cv2.copyMakeBorder(template_rotated, max_loc_y_corrected, bottom_margin, max_loc_x_corrected, right_margin, cv2.BORDER_CONSTANT, 0)

            template_mask_inverted = cv2.bitwise_not(template_mask)

            original_image_loop = cv2.bitwise_and(original_image_loop, template_mask_inverted)

        # Draw debug image
        template_matching_debug_image = cv_image.copy()
        for ((x, y), angle) in cube_positions_and_angles:
            rotated_rectangle = cv2.boxPoints(((x, y), (CUBE_SIZE, CUBE_SIZE), angle))
            rotated_rectangle = numpy.int0(rotated_rectangle)
            cv2.drawContours(template_matching_debug_image, [rotated_rectangle], -1, (0, 0, 255), 2)
        #cv2.imshow("Template matching result", template_matching_debug_image)

        return cube_positions_and_angles

def test_right_hand_ros():
    """
    Test the cube orientation sensing using ROS
    """
    rospy.init_node('cv_detection_right_hand_camera')

    camera_name = "right_hand_camera"

    camera_helper = CameraHelper(camera_name, "base", 0)

    bridge = CvBridge()

    try:
        while not rospy.is_shutdown():
            # Take picture
            img_data = camera_helper.take_single_picture()

            # Convert to OpenCV format
            cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")

            # Save for debugging
            #cv2.imwrite("/tmp/debug.png", cv_image)

            # Get cube rotation
            angles = get_cubes_z_rotation(cv_image)
            print(angles)

            # Wait for a key press
            cv2.waitKey(1)

            rospy.sleep(0.1)
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
    path = rospkg.RosPack().get_path('sorting_demo') + "/share/test_right_hand"
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
