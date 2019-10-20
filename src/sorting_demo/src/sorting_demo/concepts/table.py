class Table:
    def __init__(self):
        self.blocks = []

    def get_blocks(self):
        return self.blocks

    def get_state(self):
        return {"blocks": [b.get_state() for b in self.blocks]}

    def notify_gripper_pick(self, b,gripper):
        self.blocks.remove(b)
        gripper.holding_block = b

    def notify_gripper_place(self, b, gripper):
        gripper.holding_block = None
        self.blocks.append(b)