"""
A* 全局路径规划器
在栅格地图上搜索从起点到终点的最优路径
"""
import heapq
import numpy as np
from typing import List, Tuple, Optional


class AstarPlanner:
    """标准 A* 路径规划器，支持 4/8 邻域"""

    def __init__(self, allow_diagonal: bool = True):
        """
        Args:
            allow_diagonal: True 为 8 邻域，False 为 4 邻域
        """
        self.allow_diagonal = allow_diagonal
        if allow_diagonal:
            self._neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
            self._cost_diag = np.sqrt(2)
        else:
            self._neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            self._cost_diag = 1.0

    def plan(
        self,
        map_matrix: np.ndarray,
        start: Tuple[int, int],
        goal: Tuple[int, int],
    ) -> Optional[List[Tuple[int, int]]]:
        """
        在栅格地图上规划从 start 到 goal 的路径

        Args:
            map_matrix: 2D 数组，0=可通行，1=障碍物
            start: 起点 (row, col)
            goal: 终点 (row, col)

        Returns:
            路径点列表 [(row, col), ...]，无解时返回 None
        """
        h, w = map_matrix.shape
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))

        if not self._is_valid(map_matrix, start, h, w) or not self._is_valid(map_matrix, goal, h, w):
            return None

        # (f, g, node, parent)
        open_set = [(0, 0, start, None)]
        closed = set()
        g_map = {start: 0}
        parent_map = {start: None}

        while open_set:
            f, g, node, parent = heapq.heappop(open_set)

            if node in closed:
                continue
            closed.add(node)
            parent_map[node] = parent

            if node == goal:
                return self._reconstruct_path(parent_map, goal)

            for dr, dc in self._neighbors:
                nr, nc = node[0] + dr, node[1] + dc
                nnode = (nr, nc)
                if not self._is_valid(map_matrix, nnode, h, w) or nnode in closed:
                    continue

                cost = self._cost_diag if dr != 0 and dc != 0 else 1.0
                ng = g + cost
                if nnode not in g_map or ng < g_map[nnode]:
                    g_map[nnode] = ng
                    nf = ng + self._heuristic(nnode, goal)
                    heapq.heappush(open_set, (nf, ng, nnode, node))

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
