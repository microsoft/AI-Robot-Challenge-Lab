import copy
import re
import rospy


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
        copyfinalpose = copy.deepcopy(self.final_pose)
        copyfinalpose.position.y -= 0.19
        copyfinalpose.position.z -= 0.02
        return copyfinalpose

    def get_tray_place_block_location(self):
        #return self.final_pose
        copyfinalpose = copy.deepcopy(self.final_pose)

        yoffset = -0.1 + 0.1 * len(self.blocks)
        copyfinalpose.position.y -= yoffset
        copyfinalpose.position.z += 0.03

        rospy.logwarn("Tray Place location (objects %d, %lf): " % (len(self.blocks), yoffset) + str(copyfinalpose))

        return copyfinalpose