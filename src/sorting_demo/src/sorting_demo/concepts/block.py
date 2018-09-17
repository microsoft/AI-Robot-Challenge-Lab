import re
from rospy_message_converter import message_converter


class BlockState:
    regex = re.compile(r'block(\d+)\.*')

    def __init__(self, gazebo_id, pose):
        self.perception_id = None

        self.gazebo_id = gazebo_id
        self.pose = pose
        self.color = None
        self.homogeneous_transform = None
        search = BlockState.regex.search(gazebo_id)
        self.num = int(search.group(1))
        self.gazebo_pose = None
        self.grasp_pose = None
        self.place_pose = None


        self.table_grasp_pose = None
        self.tray_place_pose = None
        self.table_place_pose = None

        # the 2d blob point estimabion based on the head_image processing
        self.hue_estimation = None

        # the color estimation based on the head image processing
        self.hue_estimation = None

        # the 3d pose estimation from the head image processing
        self.headview_pose_estimation = None

        self.tabletop_arm_view_estimated_pose = None
        self.traytop_arm_view_estimated_pose =None

        self.tray = None



    def __str__(self):
        return "[Block estpos = %s]" % str(self.headview_pose_estimation)

    def get_state(self):
        return {"id": self.perception_id , "table_pose": message_converter.convert_ros_message_to_dictionary(self.gazebo_pose),
                "color": self.color}

    @staticmethod
    def is_block(id):
        num = BlockState.regex.search(id)
        return num is not None

    def get_color(self):
        return self.color

    def is_color(self, color):
        return color.upper() in self.color.upper()
