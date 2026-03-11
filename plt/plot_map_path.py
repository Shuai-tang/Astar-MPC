import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional, Union


def _get_map_matrix(map_data: Union[object, np.ndarray]) -> np.ndarray:
    """从 Map 实例或 ndarray 获取地图矩阵。"""
    if isinstance(map_data, np.ndarray):
        return np.asarray(map_data)
    if hasattr(map_data, "map"):
        return np.asarray(map_data.map)
    if hasattr(map_data, "get_matrix"):
        return np.asarray(map_data.get_matrix())
    raise ValueError("map_data 需提供 .map 或 .get_matrix()")


class Plotter:
    """
    栅格地图与路径绘图器。
    封装地图解析、底图绘制、起点终点与路径绘制，以及网格与坐标轴范围设置。
    """

    def __init__(
        self,
        map_data: Union[object, np.ndarray],
        show_grid: bool = True,
    ):
        """
        参数:
            map_data: Map 实例或 np.ndarray，地图数据
            show_grid: 是否显示网格
        """
        self.map_data = map_data
        self.show_grid = show_grid
        self._map_matrix = _get_map_matrix(map_data)

    def _create_figure(self):
        """创建 figure 和 axes，尺寸随地图大小自适应。"""
        h, w = self._map_matrix.shape
        fig, ax = plt.subplots(1, 1, figsize=(max(6, w / 10), max(6, h / 10)))
        return fig, ax

    def _draw_map(self, ax: plt.Axes) -> None:
        """绘制栅格地图底图。"""
        ax.imshow(
            self._map_matrix,
            cmap="binary",
            origin="upper",
            vmin=0,
            vmax=1,
        )

    def _draw_start_goal(
        self,
        ax: plt.Axes,
        start: Optional[Tuple] = None,
        goal: Optional[Tuple] = None,
    ) -> None:
        """绘制起点（绿点）和终点（红星）。"""
        if start:
            ax.plot(start[1], start[0], "go", markersize=10, zorder=4)
        if goal:
            ax.plot(goal[1], goal[0], "r*", markersize=14, zorder=4)

    def _draw_path(
        self,
        ax: plt.Axes,
        path: Optional[List[Tuple]],
        fmt: str = "b-",
        linewidth: float = 2,
        alpha: float = 0.8,
        zorder: int = 2,
    ) -> None:
        """绘制一条路径。path 格式 [(row, col), ...] 或 [(x, y), ...]。"""
        if not path or len(path) == 0:
            return
        cols = [p[1] for p in path]
        rows = [p[0] for p in path]
        ax.plot(cols, rows, fmt, linewidth=linewidth, alpha=alpha, zorder=zorder)

    def _draw_grid_and_limits(self, ax: plt.Axes) -> None:
        """设置网格与坐标轴范围。"""
        h, w = self._map_matrix.shape
        if self.show_grid:
            ax.set_xticks(np.arange(-0.5, w, 1), minor=True)
            ax.set_yticks(np.arange(-0.5, h, 1), minor=True)
            ax.grid(which="minor", color="gray", linewidth=0.3, alpha=0.5)
            ax.tick_params(which="minor", size=0)
        ax.set_xlim(-0.5, w - 0.5)
        ax.set_ylim(h - 0.5, -0.5)

    def plot_map_path(
        self,
        path: Optional[List[Tuple]] = None,
        start: Optional[Tuple] = None,
        goal: Optional[Tuple] = None,
    ) -> None:
        """
        绘制栅格地图及单条路径。

        参数:
            path: 路径列表，格式 [(row, col), ...] 或 [(x, y), ...]
            start: 起点 (row, col)
            goal: 终点 (row, col)
        """
        fig, ax = self._create_figure()
        self._draw_map(ax)
        self._draw_start_goal(ax, start, goal)
        self._draw_path(ax, path, "b-", linewidth=2, alpha=0.8, zorder=2)
        self._draw_grid_and_limits(ax)
        plt.tight_layout()
        plt.show()

    def plot_three_paths(
        self,
        astar_path: Optional[List[Tuple]] = None,
        smooth_path: Optional[List[Tuple]] = None,
        mpc_path: Optional[List[Tuple]] = None,
        start: Optional[Tuple] = None,
        goal: Optional[Tuple] = None,
    ) -> None:
        """
        在同一张图上绘制 A* 路径、平滑路径和 MPC 局部规划路径。

        参数:
            astar_path: A* 路径，格式 [(row, col), ...] 或 [(x, y), ...]
            smooth_path: 平滑后的路径（浮点坐标），格式 [(x, y), ...]
            mpc_path: MPC 路径（浮点坐标），格式 [(x, y), ...]
            start: 起点 (row, col)
            goal: 终点 (row, col)
        """
        fig, ax = self._create_figure()
        self._draw_map(ax)
        self._draw_start_goal(ax, start, goal)
        self._draw_path(ax, astar_path, "b--", linewidth=2, alpha=0.7, zorder=1)
        self._draw_path(ax, smooth_path, "g-", linewidth=2, alpha=0.8, zorder=2)
        self._draw_path(ax, mpc_path, "m-", linewidth=2.5, alpha=0.9, zorder=3)
        self._draw_grid_and_limits(ax)
        plt.tight_layout()
        plt.show()


# 兼容旧调用：保留模块级函数，内部使用 Plotter
def plot_map_path(
    map_data: Union[object, np.ndarray],
    path: Optional[List[Tuple]] = None,
    start: Optional[Tuple] = None,
    goal: Optional[Tuple] = None,
    show_grid: bool = True,
) -> None:
    """
    绘制栅格地图及路径（兼容接口）。

    参数:
        map_data: Map 实例或 np.ndarray，地图数据
        path: 路径列表，格式 [(row, col), ...] 或 [(x, y), ...]
        start: 起点 (row, col)
        goal: 终点 (row, col)
        show_grid: 是否显示网格
    """
    Plotter(map_data, show_grid=show_grid).plot_map_path(path, start, goal)


def plot_three_paths(
    map_data: Union[object, np.ndarray],
    astar_path: Optional[List[Tuple]] = None,
    smooth_path: Optional[List[Tuple]] = None,
    mpc_path: Optional[List[Tuple]] = None,
    start: Optional[Tuple] = None,
    goal: Optional[Tuple] = None,
    show_grid: bool = True,
) -> None:
    """
    在同一张图上绘制 A* 路径、平滑路径和 MPC 局部规划路径（兼容接口）。

    参数:
        map_data: Map 实例或 np.ndarray，地图数据
        astar_path: A* 路径，格式 [(row, col), ...] 或 [(x, y), ...]
        smooth_path: 平滑后的路径（浮点数坐标），格式 [(x, y), ...]
        mpc_path: MPC 路径（浮点数坐标），格式 [(x, y), ...]
        start: 起点 (row, col)
        goal: 终点 (row, col)
        show_grid: 是否显示网格
    """
    Plotter(map_data, show_grid=show_grid).plot_three_paths(
        astar_path, smooth_path, mpc_path, start, goal
    )
