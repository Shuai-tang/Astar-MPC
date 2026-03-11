import numpy as np

class Map:
    def __init__(self):
        self.map = np.zeros([60, 60])
        self._build_maze()

    def _build_maze(self):
        t = 1   # 墙体厚度
        # 地图边界
        # self.map[row, col] row:行, col:列
        self.map[0:t, :] = 1
        self.map[-t:, :] = 1
        self.map[:, 0:t] = 1
        self.map[:, -t:] = 1

        # 内部墙体
        self.map[5:60, 10:10+t] = 1
        self.map[0:55, 30:30+t] = 1
        self.map[5:60, 50:50+t] = 1

        # 中心障碍物
        self.map[15:15+t, 10:25] = 1
        self.map[30:30+t, 15:30] = 1
        self.map[45:45+t, 10:25] = 1
        self.map[25:25+t, 35:46] = 1
        self.map[40:40+t, 55:55+t] = 1
        self.map[5:5+t, 50:55] = 1
        
        