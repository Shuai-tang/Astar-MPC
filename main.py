from numpy import ma
from env.map_2 import Map
from astar import Astar
from plt import plot_map_path


def main():
    start = (55, 5)
    goal = (55, 55) 
    map = Map()
    global_planner = Astar()
    path = global_planner.plan(map.map, start, goal)
    plot_map_path(map, path=path, start=start, goal=goal)

if __name__ == "__main__":
    main()
