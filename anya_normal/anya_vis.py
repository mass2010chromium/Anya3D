from .anya_node import *
from typing import Set
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from collections import namedtuple

def draw_node(node: AnyaNode, color: str = 'r'):
    plt.plot((node.l, node.r), (node.row, node.row), 
             linestyle='-', color=color, linewidth=2, marker='.')
    if node.row != node.root[1]:
        plt.plot((node.l, node.root[0]), (node.row, node.root[1]), 
                 linestyle='--', color=color, linewidth=1, marker='.')
        plt.plot((node.r, node.root[0]), (node.row, node.root[1]),
                 linestyle='--', color=color, linewidth=1, marker='.')
    plt.plot(*node.root, marker='o',color=color)

_current_grid: Grid = None

def frame():
    global _current_grid
    plt.clf()
    plt.xlim(_current_grid.min[0], _current_grid.max[0])
    plt.ylim(_current_grid.min[1], _current_grid.max[1])
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    for cell in _current_grid.occupied:
        ax.add_patch(patches.Rectangle(cell, 1, 1, fill=True))
        

def show(interactive=False):
    plt.ion()
    plt.show()
    plt.pause(0.001)
    if interactive:
        input("Press enter to continue...")

def setup(grid: Grid, fignum: int = 0):
    global _current_grid
    plt.figure(fignum)
    _current_grid = grid

if __name__ == "__main__":
    # grid = Grid(min=(-5, -5), max=(5, 5))
    grid = makeBorderedGrid((-5, -5), (5, 5))
    node = AnyaNode((0, 0), 2, -3, 0)
    grid.occupied.update([(0, 0), (1, 1), (0, 1)])
    setup(grid)

    frame()
    draw_node(node)
    show(interactive=True)
