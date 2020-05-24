from anya_vis import *
from anya_node import *
from anya_search import *
from utils import *

import sys
import random

def test_expand_flat_right():
    grid = makeBorderedGrid((0, 0), (10, 10))
    occupied = grid.occupied
    occupied.add((2, 2))
    occupied.add((2, 3))
    occupied.add((3, 3))
    occupied.add((5, 3))
    occupied.add((7, 3))
    occupied.add((8, 3))
    occupied.add((6, 2))
    
    setup(grid)
    frame()
    #node = AnyaNode((2, 3), 3, 2, 4)
    #node = AnyaNode((2, 3), 3, 4, 5)
    node = AnyaNode((2, 2), 2, 2, 3)
    draw_node(node)
    show(interactive=True)
    
    frame()
    result_nodes = expand_flat_right(node, occupied)
    colors = ['r', 'g', 'b'] * 3
    for node, color in zip(result_nodes, colors):
        draw_node(node, color=color)
    show(interactive=True)

def test_expand_flat_left():
    grid = makeBorderedGrid((0, 0), (10, 10))
    occupied = grid.occupied
    occupied.add((1, 3))
    occupied.add((2, 3))
    occupied.add((4, 3))
    occupied.add((6, 3))
    occupied.add((7, 3))
    occupied.add((7, 2))
    occupied.add((7, 4))
    occupied.add((2, 4))
    occupied.add((3, 2))
    
    setup(grid)
    frame()
    #node = AnyaNode((8, 2), 2, 7, 8)
    #node = AnyaNode((8, 4), 4, 6, 8)
    node = AnyaNode((8, 5), 5, 7, 8)
    draw_node(node)
    show(interactive=True)
    
    frame()
    result_nodes = expand_flat_left(node, occupied)
    colors = ['r', 'g', 'b'] * 3
    for node, color in zip(result_nodes, colors):
        draw_node(node, color=color)
    show(interactive=True)

def test_project_cone():
    grid = makeBorderedGrid((0, 0), (12, 12))
    occupied = grid.occupied
    #node = AnyaNode((2, 2), 5, 6, 7)
    #node = AnyaNode((2, 8), 5, 6, 7)
    #node = AnyaNode((4, 2), 5, 2, 5)
    node = AnyaNode((11, 6), 5, 3, 4)
    #node = AnyaNode((5, 6), 5, 3, 7)

    occupied.add((1, 5))
    occupied.add((2, 5))
    occupied.add((8, 4))

    setup(grid)
    frame()
    draw_node(node)
    show(interactive=True)
    
    interval, row, check = project_cone(node, occupied)
    node2 = AnyaNode(node.root, row, *interval)

    frame()
    draw_node(node)
    draw_node(node2, color="g")
    show(interactive=True)

def test_expand_cone_up():
    grid = makeBorderedGrid((0, 0), (12, 12))
    occupied = grid.occupied
    #node = AnyaNode((4, 4), 6, 3, 6)
    node = AnyaNode((6, 4), 6, 3, 6)

    occupied.add((1, 5))
    occupied.add((2, 5))
    occupied.add((6, 5))
    occupied.add((4, 7))
    occupied.add((1, 7))
    occupied.add((2, 7))

    setup(grid)
    frame()
    draw_node(node)
    show(interactive=True)
    
    frame()
    result_nodes = expand_cone_up(node, occupied)
    print(result_nodes)
    colors = ['r', 'g', 'b'] * 3
    draw_node(node, color="gray")
    for node, color in zip(result_nodes, colors):
        draw_node(node, color=color)
    show(interactive=True)

def test_expand_cone_down():
    grid = makeBorderedGrid((0, 0), (12, 12))
    occupied = grid.occupied
    #node = AnyaNode((4, 4), 6, 3, 6)
    node = AnyaNode((6, 9), 7, 3, 7)

    occupied.add((1, 5))
    occupied.add((2, 5))
    occupied.add((6, 5))
#    occupied.add((4, 7))
    occupied.add((1, 7))
    occupied.add((2, 7))

    setup(grid)
    frame()
    draw_node(node)
    show(interactive=True)
    
    frame()
    result_nodes = expand_cone_down(node, occupied)
    print(result_nodes)
    colors = ['r', 'g', 'b'] * 3
    draw_node(node, color="gray")
    for node, color in zip(result_nodes, colors):
        draw_node(node, color=color)
    show(interactive=True)

def test_expand_start():
    grid = makeBorderedGrid((0, 0), (12, 12))
    occupied = grid.occupied

    start = (5, 6)

    occupied.add((6, 5))
    occupied.add((6, 6))
    occupied.add((4, 7))
    occupied.add((1, 7))
    occupied.add((2, 7))
    occupied.add((1, 4))
    occupied.add((2, 4))
    occupied.add((4, 4))
    occupied.add((7, 4))

    setup(grid)
    
    frame()
    result_nodes = expand_start(start, occupied)
    print(result_nodes)
    colors = ['r', 'g', 'b'] * 5
    for node, color in zip(result_nodes, colors):
        draw_node(node, color=color)
    show(interactive=True)

def test_heuristic():
    grid = makeEmptyGrid((0, 0), (12, 12))
    node = AnyaNode((5, 7), 5, 3, 6)
    
    setup(grid)
    
    frame()
    draw_node(node)
    target = (6, 2)
    result = euclidean_heuristic(node, target, vis=True)
    print(result)
    show(interactive=True)

    frame()
    draw_node(node)
    target = (6, 8)
    result = euclidean_heuristic(node, target, vis=True)
    print(result)
    show(interactive=True)

def test_random(interactive=True):
    grid = makeBorderedGrid((0, 0), (102, 102))
    occupied = grid.occupied

    target = 100*100 * 0.2
    i = 0
    while i < target:
        # Screw good rng
        pt = (random.randrange(1, 101), random.randrange(1, 101))
        if pt in occupied:
            continue
        i += 1
        occupied.add(pt)

    goal = (random.randrange(1, 101), random.randrange(1, 101))
    start = (random.randrange(1, 101), random.randrange(1, 101))
    anyaSearch = AnyaSearch()
    stime = time.time()
    anyaSearch.search_start(start, goal, grid)
    n_expanded = 0
    while not anyaSearch.search_step():
        n_expanded += 1
    
    dt = time.time() - stime
    if interactive:
        print(f"Time: {dt}")
    if interactive and not anyaSearch.has_path():
        setup(grid)
        path, distance = anyaSearch.make_path(plot=True)
    else:
        path, distance = anyaSearch.make_path()
    if interactive:
        print(path)
        print(f"Length: {distance}, Expanded {n_expanded} nodes")
        input("Press enter to exit...")
    else:
        return distance, dt, n_expanded

def test_random_multi(n=1000):
    distances, times, nnodes = zip(*(x for x in (test_random(interactive=False) for i in range(n)) if x[0] != -1))
    success = len(distances)
    print(f"Search succeeded {success}/{n} times!")
    print(f"Average path length: {sum(distances) / success}")
    print(f"Average time: {sum(times) / success}")
    print(f"Average nodes expanded: {sum(nnodes) / success}")
    plt.scatter(nnodes, times)
    plt.show()

if len(sys.argv) <= 1:
    print("Use command-line args to specify tests to run.")
for arg in sys.argv[1:]:
    print(f"Running test {arg}")
    exec(f"test_{arg}()")
    print("Done!\n")
