#!/usr/bin/env python

import json

# rospy - ROS Python API
import rospy

# intera_interface - Sawyer Python API
import intera_interface

# initialize our ROS node, registering it with the Master
rospy.init_node('Hello_Sawyer')

# create an instance of intera_interface's Limb class
limb = intera_interface.Limb('right')

# get the right limb's current joint angles
angles = limb.joint_angles()

# print the current joint angles as valid json
print json.dumps(angles)

# move to neutral pose
limb.move_to_neutral()

# get the right limb's current joint angles now that it is in neutral
angles = limb.joint_angles()

# print the current joint angles again as valid json
print json.dumps(angles)


# reassign new joint angles (all zeros) which we will later command to the limb
angles['right_j0']=0.0
angles['right_j1']=0.0
angles['right_j2']=0.0
angles['right_j3']=0.0
angles['right_j4']=0.0
angles['right_j5']=0.0
angles['right_j6']=0.0

# print the joint angle command as valid json
print json.dumps(angles)

# move the right arm to those joint angles
limb.move_to_joint_positions(angles)

# Sawyer wants to say hello, let's wave the arm

# store the first wave position 
wave_1 = {'right_j6': -1.5126, 'right_j5': -0.3438, 'right_j4': 1.5126, 'right_j3': -1.3833, 'right_j2': 0.03726, 'right_j1': 0.3526, 'right_j0': -0.4259}

# store the second wave position
wave_2 = {'right_j6': -1.5101, 'right_j5': -0.3806, 'right_j4': 1.5103, 'right_j3': -1.4038, 'right_j2': -0.2609, 'right_j1': 0.3940, 'right_j0': -0.4281}


# wave three times
for _move in range(3):
    limb.move_to_joint_positions(wave_1)
    rospy.sleep(0.5)
    limb.move_to_joint_positions(wave_2)
    rospy.sleep(0.5)
