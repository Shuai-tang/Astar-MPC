import numpy as np

class Map:
    def __init__(self):
        self.map = np.zeros([20, 20])
        self._build_maze()

    def _build_maze(self):
        # self.map[13:17,5:9] = 1
        # self.map[5:8, 3:8] = 1
        # self.map[7:12, 12:17] = 1
        self.map[1:4, 12:17] = 1