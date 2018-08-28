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

    @staticmethod
    def is_block(id):
        num = BlockState.regex.search(id)
        return num is not None

    def get_color(self):
        return self.color

    def is_color(self,color):
        return color.upper() in self.color.upper()
