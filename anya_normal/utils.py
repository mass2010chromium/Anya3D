from collections import namedtuple
from typing import Set

"""
Simple point object.

x and y should be integers( GridPoint ), not float.

This is actually just a fake and we're gonna use normal
tuples everywhere and everything has to be with [0] [1]
access but if you want to use this guy it won't break
the set data structures, just don't expect to .x .y access.
"""
GridPoint2D = namedtuple('GridPoint2D', ['x', 'y'])

"""
Object representing a grid.

Really should just have two GridPoint2D representing min and max. Max cells are excluded.
"""
Grid = namedtuple('Grid', ['min', 'max', 'occupied'])

def makeBorderedGrid(gmin: GridPoint2D, gmax: GridPoint2D):
    occupied: Set[GridPoint2D] = set()

    for i in range(gmin[0], gmax[0]):
        occupied.add(GridPoint2D(i, gmin[1]))
        occupied.add(GridPoint2D(i, gmax[1] - 1))

    for i in range(gmin[1] + 1, gmax[1] - 1):
        occupied.add(GridPoint2D(gmin[0], i))
        occupied.add(GridPoint2D(gmax[0] - 1, i))
    return Grid(gmin, gmax, occupied)

def makeEmptyGrid(gmin: GridPoint2D, gmax: GridPoint2D):
    occupied: Set[GridPoint2D] = set()
    return Grid(gmin, gmax, occupied)

def approx_eq(a, b):
    """ Returns true if the two numbers are equal within a predefined tolerance. """
    return abs(a-b) < approx_eq.eps
approx_eq.eps = 0.000001

def round_int(x):
    """ if approx_eq(x, round(x)): returns round(x) """
    rounded = round(x)
    if approx_eq(x, rounded):
        return int(rounded)
    return x

def dist2D(a, b):
    """ Euclidean distance between two points. """
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5

if __name__ == "__main__":
    test = GridPoint2D(0, 0)
    tmp = set([test])
    print(test[0], test[1])
    print(test.x, test.y)
    print(GridPoint2D(0, 0) in tmp)
    print((0, 0) in tmp)
    tmp = set([(0, 0)])
    print(GridPoint2D(0, 0) in tmp)
    print((0, 0) in tmp)
