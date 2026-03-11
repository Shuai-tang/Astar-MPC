from env.map_2 import Map
from astar import Astar
from astar.improverd_astar import Astar as Astar_improved
from plt import plot_map_path


def main():
    # 地图与起终点
    start = (55, 5)
    goal = (55, 55)
    m = Map()
    
    # ========== 全局路径规划：A* ==========
    planner = Astar()
    path = planner.plan(m.map, start, goal)

    # ========== 可视化 ==========
    plot_map_path(m, path=path, start=start, goal=goal)

def main2():
    # 地图与起终点
    start = (55, 5)
    goal = (55, 55)
    m = Map()
    
    # ========== 全局路径规划：A* ==========
    planner = Astar_improved()
    path = planner.plan(m.map, start, goal)

    # ========== 可视化 ==========
    plot_map_path(m, path=path, start=start, goal=goal)

if __name__ == "__main__":
    main()
    main2()