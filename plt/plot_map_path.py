"""
地图与路径可视化
支持 env 下 Map 类（含 get_matrix 或 .map、.start、.goal）及 A* 路径
"""
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional, Union


def plot_map_path(
    map_data: Union[object, np.ndarray],
    path: Optional[List[Tuple[int, int]]] = None,
    start: Optional[Tuple[int, int]] = None,
    goal: Optional[Tuple[int, int]] = None,
    title: str = "Map and Path",
    show_grid: bool = True,
    figsize: Optional[Tuple[float, float]] = None,
    save_path: Optional[str] = None,
) -> None:
    """
    绘制栅格地图及 A* 路径。

    参数
    ----------
    map_data : Map 实例或 np.ndarray
        - Map 实例：需有 .map 或 .get_matrix()、.start、.goal
        - np.ndarray：二维栅格，0=可通行，1=障碍；此时需同时传入 start 和 goal
    path : list of (row, col), optional
        A* 返回的路径点列表
    start : (row, col), optional
        起点，仅在 map_data 为 np.ndarray 时使用
    goal : (row, col), optional
        终点，仅在 map_data 为 np.ndarray 时使用
    title : str
        图标题
    show_grid : bool
        是否显示网格
    figsize : (w, h), optional
        图像尺寸，默认根据地图大小自动设置
    save_path : str, optional
        保存路径，若为 None 则只显示不保存
    """
    # 统一得到 map_matrix, start, goal
    if isinstance(map_data, np.ndarray):
        map_matrix = np.asarray(map_data)
        start_ = start
        goal_ = goal
    else:
        if hasattr(map_data, "map"):
            map_matrix = np.asarray(map_data.map)
        elif hasattr(map_data, "get_matrix"):
            map_matrix = np.asarray(map_data.get_matrix())
        else:
            raise ValueError("map_data 需提供 .map 或 .get_matrix()")
        start_ = tuple(map_data.start) if hasattr(map_data, "start") else start
        goal_ = tuple(map_data.goal) if hasattr(map_data, "goal") else goal
    if start is not None:
        start_ = tuple(start)
    if goal is not None:
        goal_ = tuple(goal)

    h, w = map_matrix.shape
    if figsize is None:
        figsize = (max(6, w / 40), max(6, h / 40))

    fig, ax = plt.subplots(1, 1, figsize=figsize)

    # 地图：0 白，1 黑
    cmap = plt.cm.binary
    ax.imshow(map_matrix, cmap=cmap, origin="upper", vmin=0, vmax=1)

    # 起点、终点
    if start_ is not None:
        ax.plot(start_[1], start_[0], "go", markersize=10, label="Start", zorder=3)
    if goal_ is not None:
        ax.plot(goal_[1], goal_[0], "r*", markersize=14, label="Goal", zorder=3)

    # 路径：(row, col) -> 绘图用 (x=col, y=row)
    if path and len(path) > 0:
        cols = [p[1] for p in path]
        rows = [p[0] for p in path]
        ax.plot(cols, rows, "b-", linewidth=2, alpha=0.8, label="Path", zorder=2)
        ax.plot(cols, rows, "c.", markersize=2, alpha=0.6, zorder=2)

    if show_grid:
        ax.set_xticks(np.arange(-0.5, w, 1), minor=True)
        ax.set_yticks(np.arange(-0.5, h, 1), minor=True)
        ax.grid(which="minor", color="gray", linewidth=0.3, alpha=0.5)
        ax.tick_params(which="minor", size=0)

    ax.set_xlim(-0.5, w - 0.5)
    ax.set_ylim(h - 0.5, -0.5)
    # ax.set_xlabel("col")
    # ax.set_ylabel("row")

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.show()


def plot_map_only(
    map_data: Union[object, np.ndarray],
    title: str = "Map",
    **kwargs,
) -> None:
    """仅绘制地图，不画路径。其余参数同 plot_map_path。"""
    plot_map_path(map_data, path=None, title=title, **kwargs)
