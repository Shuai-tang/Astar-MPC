# 参考自论文：基于改进A星算法融合改进动态窗口法的无人机动态避障方法研究_朱亚凯.pdf
import heapq
import numpy as np
from typing import Any, List, Tuple

class Node:
    __slots__ = ("pos", "g", "h", "d", "f")

    def __init__(self, pos: Tuple[int, int], g: float, h: float, d: float):
        self.pos = pos  # (row, col)
        self.g = g  # 从起点到当前节点的实际代价
        self.h = h  # 启发式函数值
        self.d = d  # 障碍物安全距离
        self.f = g + h + d  # g + h + d，用于堆排序

    def __lt__(self, other: "Node") -> bool:
        return self.f < other.f

class Astar:
    def __init__(self, safety_distance: int = 2):
        # 搜索方向：8个；safety_distance：搜索障碍物的栅格范围
        self._neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        self.safety_distance = safety_distance
        self.m = self.safety_distance + 1.0

    # 计算从当前节点到终点的启发式估计代价 h
    def _get_h(self, current: Tuple[int, int], goal: Tuple[int, int]) -> float:
        return np.sqrt((current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2)

    # 计算移动代价（从父节点到当前节点）
    def _get_move_cost(self, dr: int, dc: int) -> float:
        return np.sqrt(2) if dr != 0 and dc != 0 else 1.0

    # 计算节点到障碍物的代价d
    def _get_d(self, map_matrix: np.ndarray, node: Tuple[int, int], h: int, w: int):
        min_dist = self._get_min_dist(map_matrix, node, h, w, self.safety_distance)
        d = self.m - min_dist
        return d

    # 障碍物安全距离扩展函数：计算节点到最近障碍物的距离
    def _get_min_dist(self, map_matrix: np.ndarray, node: Tuple[int, int], h: int, w: int, safety_distance: int = 1) -> float:
        r, c = node
        if not (0 <= r < h and 0 <= c < w):
            return 0.0
        if map_matrix[r, c] == 1:
            return 0.0 
        # 在安全距离范围内搜索障碍物
        min_dist = float('inf')
        for dr in range(-safety_distance, safety_distance + 1):
            for dc in range(-safety_distance, safety_distance + 1):
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w:
                    if map_matrix[nr, nc] == 1:
                        dist = np.sqrt(dr**2 + dc**2)
                        min_dist = min(min_dist, dist)
        return min_dist if min_dist != float('inf') else safety_distance + 1.0 

    def plan(self, map_matrix: np.ndarray, start, goal):
        # 获取地图的行数和列数
        h, w = map_matrix.shape
        start = (start[0], start[1])
        goal = (goal[0], goal[1])

        # 判断起点和终点是否在地图内且可通行
        if not self._is_valid(map_matrix, start, h, w) or not self._is_valid(map_matrix, goal, h, w):
            return None

        # 拓展路径与障碍物相对距离系数
        d_start = self._get_d(map_matrix, start, h, w)
        
        open_set = [Node(start, 0, self._get_h(start, goal), d_start)] # 优先队列，用于存储待扩展的格子
        close_set = set[Any]() # 集合，用于存储已经扩展的格子
        g_map = {start: 0}  # 键：格子坐标；值：代价
        parent_map = {start: None}

        # while open_set 语法注释：空容器：False；非空容器：True
        while open_set:
            node = heapq.heappop(open_set) # heappop函数用于从优先队列中弹出优先级最高的节点，取出f值最小的节点
            if node.pos in close_set:
                continue
            close_set.add(node.pos)
            if node.pos == goal:
                return self._reconstruct_path(parent_map, goal)
            for dr, dc in self._neighbors:
                nr, nc = node.pos[0] + dr, node.pos[1] + dc
                npos = (nr, nc)
                # 检查邻接节点是否有效（在地图范围内且不是障碍物）且未在 closed_set 中
                if not self._is_valid(map_matrix, npos, h, w) or npos in close_set:
                    continue

                # 计算节点到障碍物的最小距离
                min_dist = self._get_min_dist(map_matrix, npos, h, w, self.safety_distance)
                
                # 如果节点太靠近障碍物（距离小于安全距离），跳过该节点
                if min_dist < self.safety_distance:
                    continue

                # 计算移动代价
                move_cost = self._get_move_cost(dr, dc)
                ng = node.g + move_cost
                
                # 计算障碍物距离代价 d
                d = self._get_d(map_matrix, npos, h, w)
                
                if npos not in g_map or ng < g_map[npos]:
                    g_map[npos] = ng
                    nh = self._get_h(npos, goal)
                    parent_map[npos] = node.pos
                    heapq.heappush(open_set, Node(npos, ng, nh, d))

        return None

    def _is_valid(self, map_matrix: np.ndarray, node: Tuple[int, int], h: int, w: int) -> bool:
        r, c = node
        return 0 <= r < h and 0 <= c < w and map_matrix[r, c] == 0

    # 回溯路径函数
    def _reconstruct_path(self, parent_map: dict, goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = parent_map[node]
        return path[::-1]
