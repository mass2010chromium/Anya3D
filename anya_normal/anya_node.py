from .utils import *

"""
An anya search node.
"""
class AnyaNode:
    def __init__(self, root: GridPoint2D, row: int, l: float, r: float):
        self.row = row
        self.root = root
        self.l = round_int(l)
        self.r = round_int(r)
        if self.row == self.root[1]:
            # Testing legal flat nodes.
            assert((self.root[0] <= self.l and self.root[0] < self.r) or 
                   (self.root[0] >= self.r and self.root[0] > self.l))

    def __str__(self):
        return f"[{self.root}, {self.row}, {self.l}, {self.r}]"

    def __repr__(self):
        return f"[{self.root}, {self.row}, {self.l}, {self.r}]"
