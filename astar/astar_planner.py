import heapq
import numpy as np
from typing import Any, List, Tuple, Optional


class Node:
    __slots__ = ("pos", "g", "f") # 用来限制类实例可以有哪些属性，用来限制内存

    def __init__(self, pos: Tuple[int, int], g: float, f: float):
        self.pos = pos  # (row, col)
        self.g = g  # 从起点到当前节点的实际代价
        self.f = f  # g + h，用于堆排序

    def __lt__(self, other: "Node") -> bool:
        return self.f < other.f

# AstarPlanner类，实现Astar算法
class AstarPlanner:
    def __init__(self):
        self._neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        self._cost_diag = np.sqrt(2) # np.sqrt函数用于求解平方根

    def plan(self,map_matrix: np.ndarray, start, goal):
        # 获取地图的行数和列数
        h, w = map_matrix.shape
        start = (start[0], start[1])
        goal = (goal[0], goal[1])

        # 判断起点和终点是否在地图内，即传入数据合理性
        if not self._is_valid(map_matrix, start, h, w) or not self._is_valid(map_matrix, goal, h, w):
            return None

        open_set = [Node(start, 0, 0)]
        closed_set = set[Any]()
        g_map = {start: 0}
        parent_map = {start: None}

        while open_set:
            node = heapq.heappop(open_set)

            if node.pos in closed_set:
                continue
            closed_set.add(node.pos)

            if node.pos == goal:
                return self._reconstruct_path(parent_map, goal)

            for dr, dc in self._neighbors:
                nr, nc = node.pos[0] + dr, node.pos[1] + dc
                npos = (nr, nc)
                if not self._is_valid(map_matrix, npos, h, w) or npos in closed_set:
                    continue

                cost = self._cost_diag if dr != 0 and dc != 0 else 1.0
                ng = node.g + cost
                if npos not in g_map or ng < g_map[npos]:
                    g_map[npos] = ng
                    nf = ng + self._heuristic(npos, goal)
                    parent_map[npos] = node.pos
                    heapq.heappush(open_set, Node(npos, ng, nf))

        return None

    def _is_valid(self, map_matrix: np.ndarray, node: Tuple[int, int], h: int, w: int) -> bool:
        r, c = node
        return 0 <= r < h and 0 <= c < w and map_matrix[r, c] == 0

    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """欧几里得距离启发式"""
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def _reconstruct_path(self, parent_map: dict, goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = parent_map[node]
        return path[::-1]
