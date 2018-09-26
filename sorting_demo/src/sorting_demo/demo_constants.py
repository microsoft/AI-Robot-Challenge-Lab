#!/usr/bin/python

import rospy
import math


def is_real_robot():
    if rospy.has_param("/use_sim_time"):
        return not rospy.get_param("/use_sim_time")
    else:
        return True

BLOCK_COLOR_MAPPINGS = [
    {"material": "Gazebo/Green"},
    {"material": "Gazebo/Red"},
    {"material": "Gazebo/Blue"},
    {"material": "Gazebo/Green"},
    {"material": "Gazebo/Red"},
    {"material": "Gazebo/Blue"},
    {"material": "Gazebo/Red"},
    {"material": "Gazebo/Blue"},
    {"material": "Gazebo/Green"}
]

TRAY_COLORS = ["Red", "Green", "Blue"]


#take into account the concept of the height of the cube
TABLE_HEIGHT_FOR_PROJECTION = -0.15

TRAY_SURFACE_THICKNESS = 0.04

ARM_TOP_VIEW_Z_OFFSET = 0.2  # meters

SIMULATE_TRAY_BLOCK_DETECTION = True

JOINT_6_OFFSET_ARM_VIEW= math.pi/2


if is_real_robot():
    APPROACH_Z_OFFSET = 0.04
    TABLE_HEIGHT = -0.08
    CUBE_EDGE_LENGTH = 0.0508
    TOOLTIP_Z_OFFSET = 0.11
else:
    APPROACH_Z_OFFSET = 0.07
    TABLE_HEIGHT_FOR_PICKING = -0.15
    CUBE_EDGE_LENGTH = 0.04
    TOOLTIP_Z_OFFSET = 0.0


