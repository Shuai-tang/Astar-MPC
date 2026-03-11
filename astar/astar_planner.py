import heapq
import numpy as np
from typing import Any, List, Tuple

class Node:
    __slots__ = ("pos", "g", "h", "f")

    def __init__(self, pos: Tuple[int, int], g: float, h: float):
        self.pos = pos  # (row, col)
        self.g = g  # 从起点到当前节点的实际代价
        self.h = h  # 启发式函数值
        self.f = g + h  # g + h，用于堆排序

    # 用于堆排序,比较价值f的大小
    def __lt__(self, other: "Node") -> bool:
        return self.f < other.f

class Astar:
    def __init__(self):
        self._neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    # 计算从起点到当前节点的实际代价 g
    def _get_g(self, parent_g: float, dr: int, dc: int) -> float:
        cost = np.sqrt(2) if dr != 0 and dc != 0 else 1.0
        return parent_g + cost

    # 计算从当前节点到终点的启发式估计代价 h
    def _get_h(self, current: Tuple[int, int], goal: Tuple[int, int]) -> float:
        return np.sqrt((current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2)

    def plan(self, map_matrix: np.ndarray, start, goal):
        # 获取地图的行数和列数
        h, w = map_matrix.shape
        start = (start[0], start[1])
        goal = (goal[0], goal[1])

        # 判断起点和终点是否在地图内且可通行
        if not self._is_valid(map_matrix, start, h, w) or not self._is_valid(map_matrix, goal, h, w):
            return None

        open_set = [Node(start, 0, self._get_h(start, goal))] # 优先队列，用于存储待扩展的格子
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

                ng = self._get_g(node.g, dr, dc)
                if npos not in g_map or ng < g_map[npos]:
                    g_map[npos] = ng
                    nh = self._get_h(npos, goal)
                    parent_map[npos] = node.pos
                    heapq.heappush(open_set, Node(npos, ng, nh))

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
