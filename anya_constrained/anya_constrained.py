from .anya2_node import *
from .anya2_vis import *
from .utils import *
from typing import Set, Callable, Tuple, Dict

from heapdict import heapdict
import matplotlib.pyplot as plt

import math
import time

def run_search(start, target, grid):
    search = Anya45Search()
    search.search_start(start, target, grid)
    n_expanded = 0
    terminate = False
    while not terminate:
        terminate, score = search.search_step()
        n_expanded += 1
    path, distance = search.make_path()
    return (path, distance, n_expanded, search)
    

"""
A more constrained version of Anya2, that restricts search nodes to be at most 45 degrees.
Needs to expand more nodes, but needs less sweeping.
"""
class Anya45Search:
    
    def __init__(self):
        pass

    def search_start(self, start: GridPoint2D, target: GridPoint2D, grid: Grid, 
                                vis: bool=False, interactive: bool=False, sleep: int=0):
        """
        Set up a search problem with the given grid, start, goal, and visualization options.
        """
        self.start = start
        self.target = target
        self.grid = grid
        self.done = False

        # Queue entries: node: score
        self.frontier = heapdict()
        # For visualization.
        self.past_nodes: Set[AnyaNode] = set()
        # Dict[to, (from, score)]
        self.roots_map = dict()

        self.vis = vis
        self.interactive = interactive
        self.sleep = sleep

        start_nodes = expand_start(self.start, self.grid.occupied, self.node_at_target)
        for node in start_nodes:
            self.frontier[node] = self.heuristic(node)

        self.roots_map[start] = (None, 0)
        if vis:
            setup(grid)
            frame()
            self.visualize()
            show(interactive=self.interactive)
            if self.sleep:
                time.sleep(self.sleep)

    def node_at_target(self, node: Anya2Node):
        """ This mutates the node! """
        if node.orientation:
            node.atTarget = (self.target[1] == node.ipos and 
                             self.target[0] >= node.lower and
                             self.target[0] <= node.upper)
        else:
            node.atTarget = (self.target[0] == node.ipos and 
                             self.target[1] >= node.lower and
                             self.target[1] <= node.upper)
        return node.atTarget

    def search_step(self):
        """
        Conduct one iteration of search.
        Returns True if the goal was found.

        Path can be reconstructed using roots_map.
        """
        if len(self.frontier) == 0:
            return True, None
        current_node, score = self.frontier.popitem()
        
        done = current_node.atTarget
        cost_to_come = self.roots_map[current_node.root][1]
        if done:
            self.roots_map[self.target] = (current_node.root, 
                                    cost_to_come + dist2D(current_node.root, self.target))
            if self.vis:
                frame()
                self.visualize()
                draw_node(current_node, color="green")
                show(interactive=self.interactive)
                if self.sleep:
                    time.sleep(self.sleep)
            self.past_nodes.add(current_node)
            new_node, new_score = self.frontier.peekitem()
            self.done = True
            if new_score >= self.roots_map[self.target][1]:
                return True, None
            return False, score
        if self.done and score >= self.roots_map[self.target][1]:
            return True, None

        if self.vis:
            frame()
            self.visualize()
            draw_node(current_node, color="red")
            show(interactive=self.interactive)
            if self.sleep:
                time.sleep(self.sleep)

        successors = expand_node(current_node, self.grid, self.node_at_target)
        first_timers = set()
        for successor in successors:
            if self.vis:
                frame()
                self.visualize()
                draw_node(current_node, color="red")
                successor_color = "orange"

            distance = dist2D(current_node.root, successor.root)
            successor_cost = cost_to_come + distance
            should_add = True
            if successor.root in self.roots_map:
                if successor_cost > self.roots_map[successor.root][1]:
                    should_add = False
                elif successor_cost == self.roots_map[successor.root][1]:
                    should_add = successor.root in first_timers or successor.root == current_node.root
                else:
                    first_timers.add(successor.root)
            else:
                first_timers.add(successor.root)
            if should_add:
                if successor.root != current_node.root:
                    self.roots_map[successor.root] = (current_node.root, successor_cost)
                heuristic_val = self.heuristic(successor, vis=self.vis)
                self.frontier[successor] = heuristic_val + successor_cost
                #successor.atTarget = self.node_at_target(successor)
            elif self.vis:
                successor_color = "blue"

            if self.vis:
                draw_node(successor, color=successor_color)
                show(interactive=self.interactive)
                if self.sleep:
                    time.sleep(self.sleep)
        self.past_nodes.add(current_node)
        return False, score

    def has_path(self):
        return self.target in self.roots_map
    
    def make_path(self, plot=False):
        if not self.has_path():
            if self.vis or plot:
                frame()
                self.visualize()
                show(interactive=self.interactive)
                if self.sleep:
                    time.sleep(self.sleep)
            return [], -1
        path = []
        current = self.target
        distance_recorded = self.roots_map[self.target][1]
        distance_measured = 0
        distance_partials = [0]
        recorded_partials = []
        while True:
            path.append(current)
            if current == self.start:
                break
            prev_node,  recorded_distance = self.roots_map[current]
            distance_measured += dist2D(current, prev_node)
            distance_partials.append(distance_measured)
            recorded_partials.append(recorded_distance)
            current = prev_node
        recorded_partials.append(0)
        # print(f"Recorded: {distance_recorded}, measured: {distance_measured}")
        path = path[::-1]
        if not (approx_eq(distance_recorded, distance_measured)):
            print("Distance error!")
            print(f"Recorded: {distance_recorded}, measured: {distance_measured}")
            if input("Diagnosis info?"):
                distance_partials = [distance_measured - x for x in distance_partials][::-1] 
                recorded_partials = recorded_partials[::-1]
                print(f"Measured: {distance_partials}")
                print(f"Recorded: {recorded_partials}")
                print(path)
                for a, b, i in zip(distance_partials, recorded_partials, range(len(distance_partials))):
                    if not approx_eq(a, b):
                        print(f"First error at index {i}: {path[i]}")
                        break
                setup(self.grid)
                frame()
                self.visualize()
                plt.plot(*zip(*path), color="green",linewidth=2,linestyle='-')
                show(interactive=True)
            if input("Stop program?"):
                import sys
                sys.exit(1)
            return path, distance_recorded
        
        if self.vis or plot:
            frame()
            self.visualize()
            plt.plot(*zip(*path), color="green",linewidth=2,linestyle='-')
            show(interactive=self.interactive)
            if self.sleep:
                time.sleep(self.sleep)
        return path, distance_recorded


    def visualize(self):
        for node in self.frontier:
            draw_node(node, color="gray")
        for node in self.past_nodes:
            draw_node(node, color="lightgray")
        plt.plot(*self.target, color="green",marker="o")
        plt.plot(*self.start, color="magenta",marker="o")

    def heuristic(self, node: Anya2Node, vis: bool = False):
        return euclidean_heuristic(node, self.target, vis=vis)
#        x = close_corner_heuristic(node, self.target) + 1
#        return euclidean_heuristic(node, self.target, vis=self.vis) * x / (x+1)
#
#def close_corner_heuristic(node: Anya2Node, target: GridPoint2D):
#    if node.orientation:
#        corner1 = (node.lower, node.ipos)
#        corner2 = (node.upper, node.ipos)
#    else:
#        corner1 = (node.ipos, node.lower)
#        corner2 = (node.ipos, node.upper)
#    return (dist2D(corner1, target) + dist2D(corner2, target))/2

def euclidean_heuristic(node: Anya2Node, target: GridPoint2D, vis: bool=False):
    if node.orientation:
        return euclidean_heuristic_vertical(node, target, vis=vis)
    else:
        # Sketchy shit
        return euclidean_heuristic_vertical(node, target, vis=vis, reflect=True)

def euclidean_heuristic_vertical(_node: Anya2Node, _target: GridPoint2D, vis: bool=False, reflect: bool=False):
    """
    Does the same calc for vertical nodes, reflecting if reflect is set.
    """
    if reflect:
        node = _node.reflect()
        target = _target[::-1]
    else:
        node = _node
        target = _target
    if vis:
        plt.plot(*_target, color='g', marker='o')

    target_dy = math.copysign(target[1] - node.ipos, node.ipos - node.root[1])

    corner = None

    if node.ipos != node.root[1]:
        dy_factor = target_dy / (node.ipos - node.root[1])
        dleft  = (node.lower - node.root[0]) * dy_factor
        dright = (node.upper - node.root[0]) * dy_factor
        ileft  = node.lower + dleft
        iright = node.upper + dright
        if target[0] <= ileft:
            corner = (node.lower, node.ipos)
        if target[0] >= iright:
            corner = (node.upper, node.ipos)
    else:
        if node.root[0] <= node.lower:
            corner = (node.lower, node.ipos)
        else: #elif node.root[0] >= node.upper:
            corner = (node.upper, node.ipos)
    if corner:
        if vis:
            if reflect:
                _corner = corner[::-1]
            else:
                _corner = corner
            plt.plot(*zip(_node.root, _corner, _target), linestyle=':', color='g', linewidth=2, marker='.')
        return dist2D(node.root, corner) + dist2D(corner, target)
    virtual_target = (target[0], node.ipos + target_dy)
    if vis:
        if reflect:
            _virtual_target = virtual_target[::-1]
        else:
            _virtual_target = virtual_target
        plt.plot(*zip(_node.root, _virtual_target), linestyle=':', color='g', linewidth=2)
        plt.plot(*zip(_target, _virtual_target), linestyle=':', color='g', linewidth=1)
    return dist2D(node.root, virtual_target)
    

def expand_start(start: GridPoint2D, occupied: Set[GridPoint2D], atTarget: Callable[[Anya2Node], bool]):

    x = start[0]
    y = start[1]

    retval = []
    if (x, y) not in occupied:
        test = Anya2Node(start, y+1, x, x+1, True)
        if atTarget(test) or (x, y+1) not in occupied:
            retval.append(test)
        test = Anya2Node(start, x+1, y, y+1, False)
        if atTarget(test) or (x+1, y) not in occupied:
            retval.append(test)
    if (x, y-1) not in occupied:
        test = Anya2Node(start, x+1, y-1, y, False)
        if atTarget(test) or (x+1, y-1) not in occupied:
            retval.append(test)
        test = Anya2Node(start, y-1, x, x+1, True)
        if atTarget(test) or (x, y-2) not in occupied:
            retval.append(test)
    if (x-1, y-1) not in occupied:
        test = Anya2Node(start, y-1, x-1, x, True)
        if atTarget(test) or (x-1, y-2) not in occupied:
            retval.append(test)
        test = Anya2Node(start, x-1, y-1, y, False)
        if atTarget(test) or (x-2, y-1) not in occupied:
            retval.append(test)
    if (x-1, y) not in occupied:
        test = Anya2Node(start, x-1, y, y+1, False)
        if atTarget(test) or (x-2, y) not in occupied:
            retval.append(test)
        test = Anya2Node(start, y+1, x-1, x, True)
        if atTarget(test) or (x-1, y+1) not in occupied:
            retval.append(test)
    return retval

def expand_node(node: Anya2Node, grid: Grid, atTarget: Callable[[Anya2Node], bool]):
    occupied: Set[GridPoint2D] = grid.occupied
    if node.orientation:
        if node.ipos > node.root[1]:
            retval = expand_cone_up(node, occupied, atTarget)
        elif node.ipos < node.root[1]:
            retval = expand_cone_down(node, occupied, atTarget)
        else:
            raise ValueError("Flat Anya2Nodes not allowed!")
    else:
        reflected = node.reflect()
        Anya2Node.make_reflected = True
        if node.ipos > node.root[0]:
            # Super sketchy mirror reflect shit. means we don't have to reflect atTarget.
            retval = expand_cone_up(reflected, ReversedOccupiedSet(occupied), atTarget)
        elif node.ipos < node.root[0]:
            retval = expand_cone_down(reflected, ReversedOccupiedSet(occupied), atTarget)
        else:
            raise ValueError("Flat Anya2Nodes not allowed!")
        Anya2Node.make_reflected = False
    if len(retval) == 1 and retval[0].root == node.root and not retval[0].atTarget:
        return expand_node(retval[0], grid, atTarget)
    return retval

def project_cone_vertical(node: Anya2Node, occupied: Set[GridPoint2D]):
    """
    Projects the node 1 unit vertically. Returns ((left, right), row, checkrow).
    Use checkrow to do splitting.

    Calculates occluded zones by iterating left and right through the grid.

    Precondition: input node is a vertical node.
    """
    dy = node.ipos - node.root[1]
    left_slope  = (node.lower - node.root[0]) / dy
    right_slope = (node.upper - node.root[0]) / dy

    delta = 1
    checkrow = node.ipos + 1
    cull_row = node.ipos
    if dy < 0:
        delta = -1
        checkrow = node.ipos - 2
        cull_row = node.ipos - 1

    ileft  = round_int(node.lower + left_slope*delta)
    iright = round_int(node.upper + right_slope*delta)

    left_cull = int(math.floor(node.lower))
    while left_cull > ileft:
        if (left_cull-1, cull_row) in occupied:
            ileft = left_cull
            break
        left_cull -= 1

    right_cull = int(math.ceil(node.upper))
    while right_cull < iright:
        if (right_cull, cull_row) in occupied:
            iright = right_cull 
            break
        right_cull += 1
    return (ileft, iright), node.ipos + delta, checkrow

def expand_cone_visible_vertical(node: Anya2Node, occupied: Set[GridPoint2D], atTarget: Callable[[Anya2Node], bool]):
    interval, row, checkrow = project_cone_vertical(node, occupied)
    if interval[0] >= interval[1]:
        # Inverted or degenerate search node. This can happen due to culling.
        # We will return the broken interval anyway and leave it up to the
        #   people downstream to not screw up.
        return [], interval
    return split_interval(node.root, row, interval[0], interval[1], checkrow, occupied, atTarget), interval

def expand_cone_up(node: Anya2Node, occupied: Set[GridPoint2D], atTarget: Callable[[Anya2Node], bool]):
    nextGridRow = node.ipos
    curGridRow = node.ipos-1
    splitGridRow = node.ipos+1
    nextIntervalRow = node.ipos+1
    return expand_cone_vertical(node, occupied, atTarget, 
                                                nextGridRow,
                                                curGridRow,
                                                splitGridRow,
                                                nextIntervalRow)

def expand_cone_down(node: Anya2Node, occupied: Set[GridPoint2D], atTarget: Callable[[Anya2Node], bool]):
    curGridRow = node.ipos
    nextGridRow = node.ipos-1
    splitGridRow = node.ipos-2
    nextIntervalRow = node.ipos-1
    return expand_cone_vertical(node, occupied, atTarget, 
                                                nextGridRow,
                                                curGridRow,
                                                splitGridRow,
                                                nextIntervalRow)

def expand_cone_vertical(node: Anya2Node, occupied: Set[GridPoint2D], 
                                          atTarget: Callable[[Anya2Node], bool], 
                                          nextGridRow: int,
                                          curGridRow: int,
                                          splitGridRow: int,
                                          nextIntervalRow: int):
    # Because an AnyaNode never contains corners, either the whole thing is wall or empty.
    # TODO: Prune earlier
    y = node.ipos
#    hit_wall = (int(math.floor(node.lower)), nextGridRow) in occupied
#    if hit_wall:
#        return []

    retval, upper_interval = expand_cone_visible_vertical(node, occupied, atTarget)
    ileft = upper_interval[0]
    iright = upper_interval[1]

    # Now we look for the non-observable neighbors.
    # Approx eq is handled during node construction, set and forget.
    if isinstance(node.upper, int):
        toCorner = (node.upper, nextGridRow) in occupied
        fromCorner = (node.upper, curGridRow) in occupied
        vertex = (node.upper, y)
        if fromCorner and not toCorner:
            # Start by going from the right boundary.
            end = node.upper + 1
            if end != iright:
                upwards_node = Anya2Node(vertex, nextIntervalRow, iright, end, True)
                if atTarget(upwards_node) or ((node.upper, splitGridRow) not in occupied):
                    retval.append(upwards_node)
            
            # Turning a corner! create an orthogonal node that goes one tile.
            corner_node = Anya2Node(vertex, node.upper+1, y, nextIntervalRow, False)
            if atTarget(corner_node) or ((node.upper+1, nextGridRow) not in occupied):
                retval.append(corner_node)

        elif toCorner and not fromCorner and iright < node.upper:
            upwards_node = Anya2Node(vertex, nextIntervalRow, iright, node.upper, True)
            if atTarget(upwards_node) or ((node.upper-1, splitGridRow) not in occupied):
                retval.append(upwards_node)

    if isinstance(node.lower, int):
        toCorner = (node.lower-1, nextGridRow) in occupied
        fromCorner = (node.lower-1, curGridRow) in occupied
        vertex = (node.lower, y)
        if fromCorner and not toCorner:
            # Start by going from the left boundary.
            end = node.lower - 1
            if end != ileft:
                upwards_node = Anya2Node(vertex, nextIntervalRow, end, ileft, True)
                if atTarget(upwards_node) or ((node.lower-1, splitGridRow) not in occupied):
                    retval.append(upwards_node)
            
            # Turning a corner! create an orthogonal node that goes one tile.
            # y = dy/dx since dx is 1.
            # but when we calculate dy/dx, dy happens to be +/-1.
            corner_node = Anya2Node(vertex, node.lower-1, y, nextIntervalRow, False)
            if atTarget(corner_node) or ((node.lower-2, nextGridRow) not in occupied):
                retval.append(corner_node)

        elif toCorner and not fromCorner and ileft > node.lower:
            upwards_node = Anya2Node(vertex, nextIntervalRow, node.lower, ileft, True)
            if atTarget(upwards_node) or ((node.lower, splitGridRow) not in occupied):
                retval.append(upwards_node)
    return retval

def split_interval(root: GridPoint2D, row: int, l: float, r: float, split_row: int,
                    occupied: Set[GridPoint2D], atTarget: Callable[[Anya2Node], bool]):
    """
    Splits an interval (l, r) which may not be integers, according to the
    wall/nonwall pattern in row split_row. Because anya nodes can't contain corners.

    Returns the resulting list of Anya2Nodes.
    """
    left_round = int(math.floor(l))
    wall_state = (left_round, split_row) in occupied

    ileft = l
    current = left_round + 1
    splits = []
    while current < r:
        # This is like the next wall section, kinda.
        current_wall = (current, split_row) in occupied
        if current_wall != wall_state:
            try_node = Anya2Node(root, row, ileft, current, True)
            if atTarget(try_node) or (not wall_state):
                splits.append(try_node)
            wall_state = current_wall
            ileft = current
        current += 1
    
    try_node = Anya2Node(root, row, ileft, r, True)
    if atTarget(try_node) or (not wall_state):
        splits.append(try_node)
    return splits

def _make_test_grid():
    grid = makeBorderedGrid((0, 0), (12, 12))
    occupied = grid.occupied

    #occupied.add((2, 7))
    occupied.add((2, 6))
    occupied.add((2, 5))
    occupied.add((2, 4))
    #occupied.add((2, 3))

    occupied.add((4, 5))
    occupied.add((5, 5))

    occupied.add((3, 3))
    occupied.add((3, 7))

    occupied.add((4, 7))
    occupied.add((4, 3))

    occupied.add((8, 2))
    occupied.add((8, 3))
    #occupied.add((8, 4))
    occupied.add((8, 5))
    occupied.add((8, 6))
    occupied.add((8, 7))
    #occupied.add((7, 7))
    occupied.add((6, 8))
    occupied.add((6, 7))
    occupied.add((6, 6))
    #occupied.add((6, 5))
    occupied.add((6, 4))
    occupied.add((6, 3))
    occupied.add((6, 2))
    occupied.add((6, 1))
    return grid


if __name__ == "__main__":
    import pickle
    grid = pickle.load(open("grid.pickl", "rb"))
#    goal = (11, 1)
#    start = (1, 1)
    goal = (82, 39)
    start = (41, 97)
    anyaSearch = Anya45Search()
    #anyaSearch.search_start(start, goal, grid, vis=True, interactive=True)
    anyaSearch.search_start(start, goal, grid, vis=True, sleep=0.025)
    #stime = time.time()
    anyaSearch.search_start(start, goal, grid)
    if anyaSearch.sleep:
        input("Press enter to start...")
    n_expanded = 0
    terminate = False
    while not terminate:
        terminate, score = anyaSearch.search_step()
        print(f"Score: {score}")
        n_expanded += 1
    setup(grid)
    path, distance = anyaSearch.make_path(plot=True)
    #path, distance = anyaSearch.make_path()
    print(path)
    #print(f"Length: {distance}, Time: {time.time() - stime}")
    print(f"Length: {distance}, Expanded {n_expanded} nodes")
    input("Press enter to exit...")
