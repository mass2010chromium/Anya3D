import anya_normal.anya_search as anya
import anya_constrained.anya_constrained as anya45
from anya_normal.utils import *

import random

def test():
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
    
    path,  distance,  nnodes,  search  = anya.run_search(start, goal, grid)
    path2, distance2, nnodes2, search2 = anya45.run_search(start, goal, grid)
    if not approx_eq(distance, distance2):
        print()
        print(f"Regression testing failed! {distance} != {distance2}")
        import anya_normal.anya_vis as anya_vis
        import anya_constrained.anya2_vis as anya45_vis
        print(f"Start: {start}, Goal: {goal}")
        print(f"Dumping grid to grid.pickl")
        import pickle
        pickle.dump(grid, open("grid.pickl", "wb"))
        anya_vis.setup(grid, 0)
        search.make_path(plot=True)
        anya45_vis.setup(grid, 1)
        search2.make_path(plot=True)
        input("Press enter to exit...")
        exit(1)
    else:
        print(".", end="", flush=True)

n = 10000
for i in range(n):
    test()
print(f"\nPassed {n} tests!")
