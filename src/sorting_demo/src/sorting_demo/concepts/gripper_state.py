class GripperState:
    def __init__(self):
        self.holding_block = None
    def get_state(self):
        return {"holding_block": None if self.holding_block is None else self.holding_block.get_state()}