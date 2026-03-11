from env.map_3 import Map
from  plt import plot_map_path
from astar.improverd_astar import Astar as Astar_improved

def main():
    start = (2, 18)
    goal = (18, 2)
    m = Map()
    planner = Astar_improved()
    path = planner.plan(m.map, start, goal)
    plot_map_path(m, path=path, start=start, goal=goal)

if __name__ == "__main__":
    main()