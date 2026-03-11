from env.map_2 import Map
from astar.improverd_astar import Astar as Astar_improved
from astar.astar_planner import Astar
from path_smooth.path_smooth import smooth_astar_path
from plt.plot_map_path import plot_map_path, plot_three_paths


def main():
    m = Map()
    start = (55, 5)
    goal = (55, 55)
    astar = Astar()
    astar_improved = Astar_improved()
    astar_path = astar.plan(m.map, start, goal)
    astar_improved_path = astar_improved.plan(m.map, start, goal)

    # 轨迹优化：对 A* 路径做平滑（样条插值 + 高斯滤波），得到可跟踪的参考轨迹
    x_smooth, y_smooth, yaw_smooth = smooth_astar_path(
        astar_improved_path,
        step_size=0.5,
        smoothing_factor=1.0,
        use_gaussian=True,
    )
    smooth_path = [(float(x), float(y)) for x, y in zip(x_smooth, y_smooth)]

    # 分别绘制：原始 A*、改进 A*、轨迹优化后的路径
    plot_map_path(m.map, astar_path, start, goal)
    plot_map_path(m.map, astar_improved_path, start, goal)
    plot_map_path(m.map, smooth_path, start, goal)

    # 可选：在一张图上对比 A* / 平滑轨迹（便于直观比较）
    plot_three_paths(
        m.map,
        astar_path=astar_improved_path,
        smooth_path=smooth_path,
        start=start,
        goal=goal,
    )


if __name__ == "__main__":
    main()