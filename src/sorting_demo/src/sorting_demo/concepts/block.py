import re


class BlockState:
    regex = re.compile(r'block(\d+)\.*')

    def __init__(self, id, pose):
        self.id = id
        self.pose = pose
        self.color = None
        self.homogeneous_transform = None
        search = BlockState.regex.search(id)
        self.num = int(search.group(1))

        # the 2d blob point estimabion based on the head_image processing
        self.hue_estimation = None

        # the color estimation based on the head image processing
        self.hue_estimation = None

        # the 3d pose estimation from the head image processing
        self.headview_pose_estimation =None

    def __str__(self):
        return "[Block estpos = %s]"%str(self.headview_pose_estimation)

    @staticmethod
    def is_block(id):
        num = BlockState.regex.search(id)
        return num is not None

    def get_color(self):
        return self.color

    def is_color(self,color):
        return color.upper() in self.color.upper()
