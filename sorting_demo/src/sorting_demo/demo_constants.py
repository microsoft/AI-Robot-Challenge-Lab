#!/usr/bin/python

import rospy

CUBE_EDGE_LENGTH = 0.04

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

TABLE_HEIGHT = -0.15

TRAY_SURFACE_THICKNESS = 0.04

ARM_TOP_VIEW_Z_OFFSET = 0.05  # meters

SIMULATE_TRAY_BLOCK_DETECTION = True


def is_real_robot():
    if rospy.has_param("/use_sim_time"):
        return not rospy.get_param("/use_sim_time")
    else:
        return False
