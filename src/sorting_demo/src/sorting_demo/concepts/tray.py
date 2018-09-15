import copy
import re

import math
import rospy
from rospy_message_converter import message_converter
import tf
import tf.transformations
from geometry_msgs.msg import Quaternion

class TrayState:
    regex = re.compile(r'tray(\d+)\.*')

    def __init__(self, id, pose, TRAY_SURFACE_THICKNESS=0.04):
        self.id = id
        self.pose = pose
        self.gazebo_pose = None
        self.TRAY_SURFACE_THICKNESS =TRAY_SURFACE_THICKNESS
        search = TrayState.regex.search(id)
        self.num = int(search.group(1))
        self.blocks = []

    @staticmethod
    def is_tray(id):
        num = TrayState.regex.search(id)
        return num is not None

    def notify_contains_block(self, block):
        self.blocks.append(block)

    def get_tray_pick_location(self):
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
        copygazebopose = copy.deepcopy(self.gazebo_pose)

        yoffset = -0.08 + 0.075 * len(self.blocks)
        copygazebopose.position.y -= yoffset
        copygazebopose.position.z += self.TRAY_SURFACE_THICKNESS

        zrot = tf.transformations.quaternion_from_euler(0, 0, -math.pi/2.0)

        trayorientatin = [copygazebopose.orientation.x, copygazebopose.orientation.y, copygazebopose.orientation.z, copygazebopose.orientation.w]
        # oorient = [overhead_orientation.x,overhead_orientation.y,overhead_orientation.z,overhead_orientation.w]

        # resultingorient = tf.transformations.quaternion_multiply(cubeorientation, tf.transformations.quaternion_conjugate(oorient))
        resultingorient = tf.transformations.quaternion_multiply(trayorientatin, zrot)

        # resultingorient = cubeorientation


        copygazebopose.orientation = Quaternion(x=resultingorient[0], y=resultingorient[1], z=resultingorient[2],
                                      w=resultingorient[3])


        rospy.logwarn("Tray Place location (objects %d, %lf): " % (len(self.blocks), yoffset) + str(copygazebopose))

        return copygazebopose

    def reset(self):
        self.blocks = []

    def get_state(self):
        return {"id": self.id, "pose":  message_converter.convert_ros_message_to_dictionary(self.gazebo_pose), "blocks": [b.get_state() for b in self.blocks]}

    def __str__(self):
        return "Tray: "+ str(self.id) +",num: "+str(self.num)+" -> "+ str(len(self.blocks)) + " items"
