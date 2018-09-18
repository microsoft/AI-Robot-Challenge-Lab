import copy
import re
import rospy
from rospy_message_converter import message_converter


class TrayState:
    regex = re.compile(r'tray(\d+)\.*')

    def __init__(self, id, pose):
        self.id = id
        self.pose = pose
        self.final_pose = None

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
        copyfinalpose = copy.deepcopy(self.final_pose)
        copyfinalpose.position.y -= 0.15
        copyfinalpose.position.z -= 0.02
        return copyfinalpose

    def get_tray_place_block_location(self):
        #return self.final_pose
        copyfinalpose = copy.deepcopy(self.final_pose)

        yoffset = -0.08 + 0.075 * len(self.blocks)
        copyfinalpose.position.y -= yoffset
        copyfinalpose.position.z += 0.03

        rospy.logwarn("Tray Place location (objects %d, %lf): " % (len(self.blocks), yoffset) + str(copyfinalpose))

        return copyfinalpose

    def reset(self):
        self.blocks = []

    def get_state(self):
        return {"id": self.id, "pose":  message_converter.convert_ros_message_to_dictionary(self.final_pose), "blocks": [b.get_state() for b in self.blocks]}

    def __str__(self):
        return "Tray: "+ str(self.id) +",num: "+str(self.num)+" -> "+ str(len(self.blocks)) + " items"
