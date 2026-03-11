import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional, Union


def plot_map_path(
    map_data: Union[object, np.ndarray],
    path: Optional[List[Tuple]] = None,
    start: Optional[Tuple] = None,
    goal: Optional[Tuple] = None,
    show_grid: bool = True,
) -> None:
    """
    绘制栅格地图及路径
    
    参数:
        map_data: Map实例或np.ndarray，地图数据
        path: 路径列表，格式 [(row, col), ...] 或 [(x, y), ...]
        start: 起点 (row, col)
        goal: 终点 (row, col)
        show_grid: 是否显示网格
    """
    # 获取地图矩阵
    if isinstance(map_data, np.ndarray):
        map_matrix = np.asarray(map_data)
    elif hasattr(map_data, "map"):
        map_matrix = np.asarray(map_data.map)
    elif hasattr(map_data, "get_matrix"):
        map_matrix = np.asarray(map_data.get_matrix())
    else:
        raise ValueError("map_data 需提供 .map 或 .get_matrix()")
    
    h, w = map_matrix.shape
    fig, ax = plt.subplots(1, 1, figsize=(max(6, w / 10), max(6, h / 10)))
    
    # 绘制地图
    ax.imshow(map_matrix, cmap='binary', origin='upper', vmin=0, vmax=1)
    
    # 绘制起点和终点
    if start:
        ax.plot(start[1], start[0], 'go', markersize=10, zorder=4)
    if goal:
        ax.plot(goal[1], goal[0], 'r*', markersize=14, zorder=4)
    
    # 绘制路径
    if path and len(path) > 0:
        cols = [p[1] for p in path]
        rows = [p[0] for p in path]
        ax.plot(cols, rows, 'b-', linewidth=2, alpha=0.8, zorder=2)
    
    # 网格设置
    if show_grid:
        ax.set_xticks(np.arange(-0.5, w, 1), minor=True)
        ax.set_yticks(np.arange(-0.5, h, 1), minor=True)
        ax.grid(which='minor', color='gray', linewidth=0.3, alpha=0.5)
        ax.tick_params(which='minor', size=0)
    
    ax.set_xlim(-0.5, w - 0.5)
    ax.set_ylim(h - 0.5, -0.5)
    
    plt.tight_layout()
    plt.show()


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
    在同一张图上绘制A*路径、平滑路径和MPC局部规划路径
    
    参数:
        map_data: Map实例或np.ndarray，地图数据
        astar_path: A*路径，格式 [(row, col), ...] 或 [(x, y), ...]
        smooth_path: 平滑后的路径（浮点数坐标），格式 [(x, y), ...]
        mpc_path: MPC路径（浮点数坐标），格式 [(x, y), ...]
        start: 起点 (row, col)
        goal: 终点 (row, col)
        show_grid: 是否显示网格
    """
    # 获取地图矩阵
    if isinstance(map_data, np.ndarray):
        map_matrix = np.asarray(map_data)
    elif hasattr(map_data, "map"):
        map_matrix = np.asarray(map_data.map)
    elif hasattr(map_data, "get_matrix"):
        map_matrix = np.asarray(map_data.get_matrix())
    else:
        raise ValueError("map_data 需提供 .map 或 .get_matrix()")
    
    h, w = map_matrix.shape
    fig, ax = plt.subplots(1, 1, figsize=(max(6, w / 10), max(6, h / 10)))
    
    # 绘制地图
    ax.imshow(map_matrix, cmap='binary', origin='upper', vmin=0, vmax=1)
    
    # 绘制起点和终点
    if start:
        ax.plot(start[1], start[0], 'go', markersize=10, zorder=4)
    if goal:
        ax.plot(goal[1], goal[0], 'r*', markersize=14, zorder=4)
    
    # 绘制A*路径（蓝色虚线）
    if astar_path and len(astar_path) > 0:
        cols = [p[1] for p in astar_path]
        rows = [p[0] for p in astar_path]
        ax.plot(cols, rows, 'b--', linewidth=2, alpha=0.7, zorder=1)
    
    # 绘制平滑路径（绿色实线）
    if smooth_path and len(smooth_path) > 0:
        cols = [p[1] for p in smooth_path]
        rows = [p[0] for p in smooth_path]
        ax.plot(cols, rows, 'g-', linewidth=2, alpha=0.8, zorder=2)
    
    # 绘制MPC路径（品红色实线）
    if mpc_path and len(mpc_path) > 0:
        cols = [p[1] for p in mpc_path]
        rows = [p[0] for p in mpc_path]
        ax.plot(cols, rows, 'm-', linewidth=2.5, alpha=0.9, zorder=3)
    
    # 网格设置
    if show_grid:
        ax.set_xticks(np.arange(-0.5, w, 1), minor=True)
        ax.set_yticks(np.arange(-0.5, h, 1), minor=True)
        ax.grid(which='minor', color='gray', linewidth=0.3, alpha=0.5)
        ax.tick_params(which='minor', size=0)
    
    ax.set_xlim(-0.5, w - 0.5)
    ax.set_ylim(h - 0.5, -0.5)
    
    plt.tight_layout()
    plt.show()
