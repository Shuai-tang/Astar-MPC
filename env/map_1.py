import numpy as np


class Map:
    def __init__(self):
        self.map = np.zeros([60, 60])
        self._obstacles = []  # [(center, radius), ...]
        self._build_maze()

    def _build_maze(self):
        # 圆心障碍物：(行, 列), 半径（栅格数）
        self._build_ratio(center=(30, 30), radius=8)
        self._build_ratio(center=(15, 45), radius=5)
        self._build_ratio(center=(45, 15), radius=6)

    def _build_ratio(self, center, radius):
        self._obstacles.append((center, radius))
        r0, c0 = center
        h, w = self.map.shape
        for i in range(h):
            for j in range(w):
                if (i - r0) ** 2 + (j - c0) ** 2 <= radius**2:
                    self.map[i, j] = 1

    def get_obstacle(self):
        """返回障碍物列表，每项为 (圆心, 半径)，圆心为 (row, col)。"""
        return self._obstacles