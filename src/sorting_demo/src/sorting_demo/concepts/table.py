class Table:
    def __init__(self):
        self.blocks = []

    def get_blocks(self):
        return self.blocks

    def get_state(self):
        return {"blocks": [b.get_state() for b in self.blocks]}

    def notify_block_removed(self, b):
        self.blocks.remove(b)