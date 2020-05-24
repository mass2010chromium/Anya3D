from .anya_node import *
from .anya_vis import *
from .utils import *
from typing import Set, Callable, Tuple, Dict

from heapdict import heapdict
import matplotlib.pyplot as plt

import math
import time

def run_search(start, target, grid):
    search = AnyaSearch()
    search.search_start(start, target, grid)
    n_expanded = 0
    terminate = False
    while not terminate:
        terminate = search.search_step()
        n_expanded += 1
    path, distance = search.make_path()
    return (path, distance, n_expanded, search)

class AnyaSearch:
    
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

        start_nodes = expand_start(self.start, self.grid.occupied)
        for node in start_nodes:
            self.frontier[node] = euclidean_heuristic(node, self.target, vis)

        self.roots_map[start] = (None, 0)
        if vis:
            setup(grid)
            frame()
            self.visualize()
            show(interactive=self.interactive)
            if self.sleep:
                time.sleep(self.sleep)

    def search_step(self):
        """
        Conduct one iteration of search.
        Returns True if the goal was found.

        Path can be reconstructed using roots_map.
        """
        if len(self.frontier) == 0:
            return True
        current_node, score = self.frontier.popitem()
        cost_to_come = self.roots_map[current_node.root][1]
        if (self.target[1] == current_node.row and 
                self.target[0] >= current_node.l and
                self.target[0] <= current_node.r):
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
            return True

        if self.vis:
            frame()
            self.visualize()
            draw_node(current_node, color="red")
            show(interactive=self.interactive)
            if self.sleep:
                time.sleep(self.sleep)

        successors = expand_node(current_node, self.grid)
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
                heuristic_val = euclidean_heuristic(successor, self.target, vis=self.vis)
                self.frontier[successor] = heuristic_val + successor_cost
            elif self.vis:
                successor_color = "blue"

            if self.vis:
                draw_node(successor, color=successor_color)
                show(interactive=self.interactive)
                if self.sleep:
                    time.sleep(self.sleep)
        self.past_nodes.add(current_node)
        return False

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

def euclidean_heuristic(node: AnyaNode, target: GridPoint2D, vis: bool=False):
    if vis:
        plt.plot(*target, color='g', marker='o')

    target_dy = math.copysign(target[1] - node.row, node.row - node.root[1])
    corner = None
    if node.row != node.root[1]:
        dy_factor = target_dy / (node.row - node.root[1])
        dleft  = (node.l - node.root[0]) * dy_factor
        dright = (node.r - node.root[0]) * dy_factor
        ileft = node.l + dleft
        iright = node.r + dright
    
        if target[0] <= ileft:
            corner = (node.l, node.row)
        if target[0] >= iright:
            corner = (node.r, node.row)
    else:
        if node.root[0] <= node.l:
            corner = (node.l, node.row)
        else: #elif node.root[0] >= node.r:
            corner = (node.r, node.row)
    if corner:
        if vis:
            plt.plot(*zip(node.root, corner, target), linestyle=':', color='g', linewidth=2, marker='.')
        return dist2D(node.root, corner) + dist2D(corner, target)
    if vis:
        plt.plot(*zip(node.root, (target[0], node.row + target_dy)), 
                    linestyle=':', color='g', linewidth=2)
        plt.plot(*zip(target, (target[0], node.row + target_dy)), 
                    linestyle=':', color='g', linewidth=1)
    return dist2D(node.root, (target[0], node.row + target_dy))
    

def expand_start(start: GridPoint2D, occupied: Set[GridPoint2D]):
    x = start[0]
    y = start[1]
    top_left     = probe_horizontal_simple(x-1, y,   -1, occupied) + 1
    top_right    = probe_horizontal_simple(x,   y,    1, occupied)
    bottom_left  = probe_horizontal_simple(x-1, y-1, -1, occupied) + 1
    bottom_right = probe_horizontal_simple(x,   y-1,  1, occupied)

    retval = []
    topRCorner = (x  , y  ) in occupied
    topLCorner = (x-1, y  ) in occupied
    botRCorner = (x  , y-1) in occupied
    botLCorner = (x-1, y-1) in occupied
    if topRCorner or botRCorner:
        if not topRCorner:
            # Only wall on bottom right. Scan until we are blocked, or no more bottom wall.
            end = probe_horizontal(x+1, y, 1, 
                                    lambda x, y: (x, y) in occupied or not (x, y-1) in occupied)
            retval.append(AnyaNode(start, y, x, end))
        if not botRCorner:
            # Only wall on top right. Scan until we are blocked, or no more top wall.
            end = probe_horizontal(x+1, y, 1, 
                                    lambda x, y: not (x, y) in occupied or (x, y-1) in occupied)
            retval.append(AnyaNode(start, y, x, end))
    else:
        # Right side unobstructed.
        retval.append(AnyaNode(start, y, x, min(top_right, bottom_right)))
    if topLCorner or botLCorner:
        if not topLCorner:
            # Only wall on bottom right. Scan until we are blocked, or no more bottom wall.
            end = probe_horizontal(x-2, y, -1, 
                                    lambda x, y: (x, y) in occupied or not (x, y-1) in occupied)
            retval.append(AnyaNode(start, y, end+1, x))
        if not botLCorner:
            # Only wall on top right. Scan until we are blocked, or no more top wall.
            end = probe_horizontal(x-2, y, -1, 
                                    lambda x, y: not (x, y) in occupied or (x, y-1) in occupied)
            retval.append(AnyaNode(start, y, end+1, x))
    else:
        # Right side unobstructed.
        retval.append(AnyaNode(start, y, max(top_left, bottom_left), x))
    
    if top_left != top_right:
        retval += [AnyaNode(start, y+1, split[0], split[1]) 
                        for split in split_interval(top_left, top_right, y+1, occupied)]
    if bottom_left != bottom_right:
        retval += [AnyaNode(start, y-1, split[0], split[1]) 
                        for split in split_interval(bottom_left, bottom_right, y-2, occupied)]
    return retval

def expand_node(node: AnyaNode, grid: Grid):
    occupied: Set[GridPoint2D] = grid.occupied
    if node.row == node.root[1]:
        return expand_flat(node, occupied)
    if node.row > node.root[1]:
        return expand_cone_up(node, occupied)
    else: #elif node.row < node.root[1]:
        return expand_cone_down(node, occupied)

def project_cone(node: AnyaNode, occupied: Set[GridPoint2D]):
    """
    Projects the node 1 unit outwards. Returns ((left, right), row, checkrow).
    Use checkrow to do splitting.

    Calculates occluded zones by iterating left and right through the grid.
    """
    dy = node.row - node.root[1]
    left_slope  = (node.l - node.root[0]) / dy
    right_slope = (node.r - node.root[0]) / dy

    delta = 1
    checkrow = node.row + 1
    cull_row = node.row
    if dy < 0:
        delta = -1
        checkrow = node.row - 2
        cull_row = node.row - 1

    ileft  = round_int(node.l + left_slope*delta)
    iright = round_int(node.r + right_slope*delta)

    left_cull = int(math.floor(node.l))
    while left_cull > ileft:
        if (left_cull-1, cull_row) in occupied:
            ileft = left_cull
            break
        left_cull -= 1

    right_cull = int(math.ceil(node.r))
    while right_cull < iright:
        if (right_cull, cull_row) in occupied:
            iright = right_cull 
            break
        right_cull += 1
    return (ileft, iright), node.row + delta, checkrow

def expand_cone_visible(node: AnyaNode, occupied: Set[GridPoint2D]):
    interval, row, checkrow = project_cone(node, occupied)
    if interval[0] >= interval[1]:
        # Inverted or degenerate search node. This can happen due to culling.
        # We will return the broken interval anyway and leave it up to the
        #   people downstream to not screw up.
        return [], interval
    splits = split_interval(interval[0], interval[1], checkrow, occupied)
    return [AnyaNode(node.root, row, split[0], split[1]) for split in splits], interval

def expand_cone_up(node: AnyaNode, occupied: Set[GridPoint2D]):
    return expand_cone(node, occupied, node.row, node.row-1, node.row+1, node.row+1)

def expand_cone_down(node: AnyaNode, occupied: Set[GridPoint2D]):
    return expand_cone(node, occupied, node.row-1, node.row, node.row-2, node.row-1)

def expand_cone(node: AnyaNode, occupied: Set[GridPoint2D], nextGridRow: int,
                                                            curGridRow: int,
                                                            splitGridRow: int,
                                                            nextIntervalRow: int):
    # Because an AnyaNode never contains corners, either the whole thing is wall or empty.
    # TODO: Prune earlier
    y = node.row
    hit_wall = (int(math.floor(node.l)), nextGridRow) in occupied
    if hit_wall:
        return []

    retval, upper_interval = expand_cone_visible(node, occupied)
    ileft = upper_interval[0]
    iright = upper_interval[1]

    # Now we look for the non-observable neighbors.
    # Approx eq is handled during node construction, set and forget.
    if isinstance(node.r, int):
        toCorner = (node.r, nextGridRow) in occupied
        fromCorner = (node.r, curGridRow) in occupied
        vertex = (node.r, y)
        if fromCorner and not toCorner:
            # We only need to add new flat nodes if it "wraps around" like this.
            # Turning a corner!
            # Start at the upper right corner of the "corner" block.
            end = probe_horizontal(node.r+1, y, 1, 
                                    lambda x, y: (x, nextGridRow) in occupied or not (x, curGridRow) in occupied)
            retval.append(AnyaNode(vertex, y, node.r, end))

            # Start by going from the right boundary. Go until we hit a solid wall.
            end = probe_horizontal_simple(int(math.ceil(iright)), nextGridRow, 1, occupied)
            # This if statement filters out culled, degenerate, or inverted nodes, 
            #   where iright will already have hit a wall.
            if end != iright:
                splits = split_interval(iright, end, splitGridRow, occupied)
                for split in splits:
                    retval.append(AnyaNode(vertex, nextIntervalRow, split[0], split[1]))

        elif toCorner and not fromCorner and iright < node.r:
            # This max() call is needed in case of inverted search nodes.
            # Using information about ileft means we don't have to repeat the
            #   leftwards scan-until-wall-or-iright.
            splits = split_interval(max(ileft, iright), node.r, splitGridRow, occupied)
            for split in splits:
                retval.append(AnyaNode(vertex, nextIntervalRow, split[0], split[1]))

    if isinstance(node.l, int):
        toCorner = (node.l-1, nextGridRow) in occupied
        fromCorner = (node.l-1, curGridRow) in occupied
        vertex = (node.l, y)
        if fromCorner and not toCorner:
            # We only need to add new flat nodes if it "wraps around" like this.
            # Turning a corner!
            # Start at the upper left corner of the "corner" block.
            end = probe_horizontal(node.l-2, y, -1, 
                                    lambda x, y: (x, nextGridRow) in occupied or not (x, curGridRow) in occupied)
            retval.append(AnyaNode(vertex, y, end+1, node.l))

            # Start by going from the left boundary. Go until we hit a solid wall.
            end = probe_horizontal_simple(int(math.floor(ileft-1)), nextGridRow, -1, occupied)
            # This if statement filters out culled, degenerate, or inverted nodes, 
            #   where ileft will already have hit a wall.
            if end+1 != ileft:
                splits = split_interval(end+1, ileft, splitGridRow, occupied)
                for split in splits:
                    retval.append(AnyaNode(vertex, nextIntervalRow, split[0], split[1]))

        elif toCorner and not fromCorner and ileft > node.l:
            # This min() call is needed in case of inverted search nodes.
            # Using information about ileft means we don't have to repeat the
            #   leftwards scan-until-wall-or-iright.
            splits = split_interval(node.l, min(ileft, iright), splitGridRow, occupied)
            for split in splits:
                retval.append(AnyaNode(vertex, nextIntervalRow, split[0], split[1]))
    return retval


def expand_flat(node: AnyaNode, occupied: Set[GridPoint2D]):
    """ Expand a flat node in Anya search. """
    y = node.row
    if node.root[0] <= node.l:
        # Expand rightwards.
        retval = expand_flat_right(node, occupied)
    elif node.root[0] >= node.r:
        # Expand rightwards.
        retval = expand_flat_left(node, occupied)
    else:
        raise ValueError(f"Invalid flat AnyaNode encountered! {node}")
    return retval

def probe_horizontal(x: int, y: int, direction: int, stop: Callable[[int, int], bool]):
    """
    Probe starting from the given point in the given direction until stop() returns true.
    """
    while not stop(x, y):
        x += direction
    return x

def probe_horizontal_simple(x: int, y: int, direction: int, occupied: Set[GridPoint2D]):
    """
    Probe starting from the given point in the given direction until a wall is hit.
    Grid cell coordinates, not vertex coordinates.
    """
    while not (x, y) in occupied:
        x += direction
    return x

def split_interval(l: float, r: float, split_row: int, occupied: Set[GridPoint2D]):
    """
    Splits an interval (l, r) which may not be integers, according to the
    wall/nonwall pattern in row split_row. Because anya nodes can't contain corners.

    Returns a list of tuples (left, right).
    """
    left_round = math.floor(l)
    wall_state = (left_round, split_row) in occupied

    ileft = l
    current = left_round + 1
    splits = []
    while current < r:
        # This is like the next wall section, kinda.
        current_wall = (current, split_row) in occupied
        if current_wall != wall_state:
            splits.append((ileft, current))
            wall_state = current_wall
            ileft = current
        current += 1
    splits.append((ileft, r))
    return splits
    

def expand_flat_right(node: AnyaNode, occupied: Set[GridPoint2D]):
    """
    Expand a flat node rightwards in Anya search.
    Helper function for expand_flat(AnyaNode, Set[GridPoint2D]).
    
    Precondition: node.root[0] <= node.l (The node root is to the left of the node itself.)
    """
    retval = []
    y = node.row
    start = node.r

    # Question: Can start be non integral? I think not, due to the way flat nodes are generated.
    #   (They are generated when someone coming from the left "rounds a corner".)
    #   If not we just fix it by bumping the "current" one to the right.
    #   
    #   current = int(math.ceil(start))
    assert(start == int(start))
    
    # By default we make collision checks for both.
    checkBottom = True
    checkTop = True

    if (start-1, y) in occupied:
        # The grid cell that is the upper left of the start point
        #   is occupied. This means we are a "wall-hugging" cell.
        checkTop = False
        
        if (start, y-1) in occupied:
            # We are blocked in.
            return []

        # There will also be at least one new node formed by going around the corner.
        # TODO: Add pruning here.
        
        end = probe_horizontal_simple(start+1, y, 1, occupied)
        # Now cur2 is the bottom left of the first wall we ran into.
        splits = split_interval(start, end, y+1, occupied)
        for split in splits:
            retval.append(AnyaNode((start, y), y+1, split[0], split[1]))
        
    if (start-1, y-1) in occupied:
        # The grid cell that is the bottom left of the start point
        #   is occupied. This means we are a "wall-hugging" cell.
        checkBottom = False

        if (start, y) in occupied:
            # We are blocked in.
            return []

        # There will also be at least one new node formed by going around the corner.
        # TODO: Add pruning here.
        
        end = probe_horizontal_simple(start+1, y-1, 1, occupied)
        # Now cur2 is the bottom left of the first wall we ran into.
        splits = split_interval(start, end, y-2, occupied)
        for split in splits:
            retval.append(AnyaNode((start, y), y-1, split[0], split[1]))
        
    assert(checkTop or checkBottom) # Check that we can't be coming from both TL and BL occupied.

    # ------------------------------------------------------------
    # 
    # This part adds a new flat node that extends to the right.
    # 
    # ------------------------------------------------------------

    def stop_probe(x: int, y: int):
        if not stop_probe.emptyStart:
            return (x, y-1) in occupied or (x, y) in occupied

        retval = stop_probe.seenBottom or stop_probe.seenTop
        if (x, y-1) in occupied:
            stop_probe.seenBottom = True
            retval = stop_probe.seenTop
        if (x, y) in occupied:
            stop_probe.seenTop = True
            retval = stop_probe.seenBottom
        return retval
    stop_probe.emptyStart = checkTop and checkBottom
    stop_probe.seenBottom = False
    stop_probe.seenTop = False

    end = probe_horizontal(start, y, 1, stop_probe)

    # This one might terminate immediately if it "runs into a wall". 
    # I'm not checking for that early, maybe I should.
    if end != start:
        # TODO: Add pruning here.
        retval.append(AnyaNode(node.root, y, start, end))
    
    return retval

def expand_flat_left(node: AnyaNode, occupied: Set[GridPoint2D]):
    """
    Expand a flat node leftwards in Anya search.
    Helper function for expand_flat(AnyaNode, Set[GridPoint2D]).
    
    Precondition: node.root[0] >= node.r (The node root is to the right of the node itself.)
    """
    retval = []
    y = node.row
    start = node.l

    # Question: Can start be non integral? I think not, due to the way flat nodes are generated.
    #   (They are generated when someone coming from the right "rounds a corner".)
    #   If not we just fix it by bumping the "current" one to the left.
    #   
    #   current = int(math.ceil(start))
    assert(start == int(start))
    
    # By default we make collision checks for both.
    checkBottom = True
    checkTop = True

    if (start, y) in occupied:
        # The grid cell that is the upper right of the start point
        #   is occupied. This means we are a "wall-hugging" cell.
        checkTop = False

        if (start-1, y-1) in occupied:
            # We are blocked in.
            return []

        # There will also be at least one new node formed by going around the corner.
        # TODO: Add pruning here.
        
        end = probe_horizontal_simple(start-2, y, -1, occupied)
        # Now cur2 is the bottom left of the first wall we ran into.
        splits = split_interval(end+1, start, y+1, occupied)
        for split in splits:
            retval.append(AnyaNode((start, y), y+1, split[0], split[1]))
        
    if (start, y-1) in occupied:
        # The grid cell that is the bottom right of the start point
        #   is occupied. This means we are a "wall-hugging" cell.
        checkBottom = False

        if (start-1, y) in occupied:
            # We are blocked in.
            return []

        # There will also be at least one new node formed by going around the corner.
        # TODO: Add pruning here.
        
        end = probe_horizontal_simple(start-2, y-1, -1, occupied)
        # Now cur2 is the bottom left of the first wall we ran into.
        splits = split_interval(end+1, start, y-2, occupied)
        for split in splits:
            retval.append(AnyaNode((start, y), y-1, split[0], split[1]))
        
    assert(checkTop or checkBottom) # Check that we can't be coming from both TL and BL occupied.

    # ------------------------------------------------------------
    # 
    # This part adds a new flat node that extends to the left.
    # 
    # ------------------------------------------------------------

    def stop_probe(x: int, y: int):
        if not stop_probe.emptyStart:
            return (x-1, y-1) in occupied or (x-1, y) in occupied

        retval = stop_probe.seenBottom or stop_probe.seenTop
        if (x-1, y-1) in occupied:
            stop_probe.seenBottom = True
            retval = stop_probe.seenTop
        if (x-1, y) in occupied:
            stop_probe.seenTop = True
            retval = stop_probe.seenBottom
        return retval
    stop_probe.emptyStart = checkTop and checkBottom
    stop_probe.seenBottom = False
    stop_probe.seenTop = False

    end = probe_horizontal(start, y, -1, stop_probe)

    # This one might terminate immediately if it "runs into a wall". 
    # I'm not checking for that early, maybe I should.
    if end != start:
        # TODO: Add pruning here.
        retval.append(AnyaNode(node.root, y, end, start))
    
    return retval

if __name__ == "__main__":
    grid = makeBorderedGrid((0, 0), (12, 12))
    occupied = grid.occupied

    occupied.add((2, 7)) # Corner toggle
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
    occupied.add((8, 4))
    occupied.add((8, 5))
    occupied.add((8, 6))
    occupied.add((8, 7))
    #occupied.add((7, 7))
    occupied.add((6, 7))
    occupied.add((6, 6))
    #occupied.add((6, 5))
    occupied.add((6, 4))
    occupied.add((6, 3))
    occupied.add((6, 2))
    occupied.add((6, 1))

    goal = (11, 1)
    start = (1, 1)
    anyaSearch = AnyaSearch()
    #anyaSearch.search_start(start, goal, grid, vis=True, interactive=True)
    #anyaSearch.search_start(start, goal, grid, vis=True, sleep=0.025)
    stime = time.time()
    anyaSearch.search_start(start, goal, grid)
    n_expanded = 0
    while not anyaSearch.search_step():
        n_expanded += 1
    #setup(grid)
    #path, distance = anyaSearch.make_path(plot=True)
    path, distance = anyaSearch.make_path()
    print(path)
    print(f"Length: {distance}, Time: {time.time() - stime}")
    print(f"Length: {distance}, Expanded {n_expanded} nodes")
    input("Press enter to exit...")
