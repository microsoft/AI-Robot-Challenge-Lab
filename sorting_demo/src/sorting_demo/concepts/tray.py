import copy
import re

import math
import rospy
from rospy_message_converter import message_converter
import tf
import tf.transformations
from geometry_msgs.msg import Quaternion
import demo_constants

class TrayState:
    regex = re.compile(r'tray(\d+)\.*')

    def __init__(self, id, pose, TRAY_SURFACE_THICKNESS=0.04):
        self.gazebo_id = id
        self.pose = pose
        self.gazebo_pose = None
        self.TRAY_SURFACE_THICKNESS =TRAY_SURFACE_THICKNESS

        if not demo_constants.is_real_robot():
            search = TrayState.regex.search(id)
            self.num = int(search.group(1))
        else:
            self.num = int(id)

        self.blocks = []

    @staticmethod
    def is_tray(id):
        num = TrayState.regex.search(id)
        return num is not None

    def notify_place_block(self, block, gripper_state):
        block.tray = self
        self.blocks.append(block)
        gripper_state.holding_block = None


    def notify_pick_block(self, block, gripper_state):
        block.tray = None
        self.blocks.remove(block)
        gripper_state.holding_block = block

    def get_tray_pick_location_for_turning_over(self):
        """
        provides the grasping pose for the tray
        
        :return: geometry_msg/Pose  
        """
        copyfinalpose = copy.deepcopy(self.gazebo_pose)
        copyfinalpose.position.y -= 0.15
        copyfinalpose.position.z -= 0.02
        return copyfinalpose

    def get_tray_place_block_pose(self):
        #return self.gazebo_pose

        if demo_constants.is_real_robot:
            copygazebopose = copy.deepcopy(self.pose)
        else:
            copygazebopose = copy.deepcopy(self.gazebo_pose)

        yoffset = -0.08 + demo_constants.TRAY_CUBEi_OFFSET_FACTOR * len(self.blocks)
        copygazebopose.position.y -= yoffset
        copygazebopose.position.z += self.TRAY_SURFACE_THICKNESS


        if not demo_constants.is_real_robot():
            angle = -math.pi / 2.0
        else:
            angle = math.pi / 2.0

        zrot = tf.transformations.quaternion_from_euler(0, 0, angle)

        trayorientatin = [copygazebopose.orientation.x, copygazebopose.orientation.y, copygazebopose.orientation.z, copygazebopose.orientation.w]
        # oorient = [overhead_orientation.x,overhead_orientation.y,overhead_orientation.z,overhead_orientation.w]

        # resultingorient = tf.transformations.quaternion_multiply(cubeorientation, tf.transformations.quaternion_conjugate(oorient))
        resultingorient = tf.transformations.quaternion_multiply(trayorientatin, zrot)

        # resultingorient = cubeorientation


        copygazebopose.orientation = Quaternion(x=resultingorient[0], y=resultingorient[1], z=resultingorient[2],
                                      w=resultingorient[3])


        rospy.logwarn("Tray Place location (objects %d, %lf): " % (len(self.blocks), yoffset) + str(copygazebopose))

        return copygazebopose


    def get_state(self):
        return {"pose":  message_converter.convert_ros_message_to_dictionary(self.gazebo_pose), "blocks": [b.get_state() for b in self.blocks]}

    def __str__(self):
        return "Tray: "+ str(self.gazebo_id) + ",num: " + str(self.num) + " -> " + str(len(self.blocks)) + " items"
