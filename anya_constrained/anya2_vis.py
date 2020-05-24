from .anya2_node import *
from typing import Set
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from collections import namedtuple

def draw_node(node: Anya2Node, color: str = 'r'):
    if node.orientation:
        plt.plot((node.lower, node.upper), (node.ipos, node.ipos), 
                 linestyle='-', color=color, linewidth=2, marker='.')
        plt.plot((node.lower, node.root[0]), (node.ipos, node.root[1]), 
                 linestyle='--', color=color, linewidth=1, marker='.')
        plt.plot((node.upper, node.root[0]), (node.ipos, node.root[1]),
                 linestyle='--', color=color, linewidth=1, marker='.')
    else:
        plt.plot((node.ipos, node.ipos), (node.lower, node.upper), 
                 linestyle='-', color=color, linewidth=2, marker='.')
        plt.plot((node.ipos, node.root[0]), (node.lower, node.root[1]), 
                 linestyle='--', color=color, linewidth=1, marker='.')
        plt.plot((node.ipos, node.root[0]), (node.upper, node.root[1]),
                 linestyle='--', color=color, linewidth=1, marker='.')
    plt.plot(*node.root, marker='o',color=color)

_current_grid: Grid = None

def frame():
    global _current_grid
    ax = plt.gca()
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    plt.clf()
    ax = plt.gca()
    plt.xlim(*xlim)
    plt.ylim(*ylim)
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
    plt.xlim(_current_grid.min[0], _current_grid.max[0])
    plt.ylim(_current_grid.min[1], _current_grid.max[1])

if __name__ == "__main__":
    grid = makeBorderedGrid((-5, -5), (5, 5))
    grid.occupied.update([(0, 0), (1, 1), (0, 1)])
    setup(grid)

    frame()
    node = Anya2Node((0, 1), 2, -3, 0, True)
    draw_node(node)
    node = node.reflect()
    draw_node(node, color='g')
    show(interactive=True)
