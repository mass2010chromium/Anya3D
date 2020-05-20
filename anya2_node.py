from utils import *

"""
An anya2 search node. These guys are allowed to be horizontal too.

orientation: True if vertical orientation (normal), False if horizontal orientation.
"""
class Anya2Node:
    def __init__(self, root: GridPoint2D, ipos: int, a: float, b: float, orientation: bool, atTarget: bool=False):
        self.ipos = ipos
        if Anya2Node.make_reflected:
            self.root = (root[1], root[0])
        else:
            self.root = root
        self.lower = round_int(min(a, b))
        self.upper = round_int(max(a, b))
        self.orientation = orientation ^ Anya2Node.make_reflected
        self.atTarget = atTarget

        if Anya2Node.construction_warn:
            if a == b:
                print(f"Warning: Degenerate Anya2Node constructed! ({self})")
            if orientation:
                check = root[1]
            else:
                check = root[0]
            if check == ipos:
                print(f"Warning: Flat nodes are not allowed or handled properly in anya2! ({self})")

    def reflect(self):
        return Anya2Node(self.root[::-1], self.ipos, self.lower, self.upper, not self.orientation, self.atTarget)

    def __str__(self):
        return f"[{self.root}, {self.ipos}, {self.lower}, {self.upper}, {self.orientation}]"

    def __repr__(self):
        return f"[{self.root}, {self.ipos}, {self.lower}, {self.upper}, {self.orientation}]"
Anya2Node.construction_warn = True
Anya2Node.make_reflected = False
