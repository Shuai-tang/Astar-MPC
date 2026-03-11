import numpy as np

class Map:
    def __init__(self):
        self.map = np.zeros([500, 500])
        self.start = [475, 25]
        self.goal = [475, 475]
        self._build_maze()
        
    def _build_maze(self):
        t = 6   # 墙体厚度
        # 地图边界
        self.map[0:t, :] = 1
        self.map[-t:, :] = 1
        self.map[:, 0:t] = 1
        self.map[:, -t:] = 1

        # 内部墙体
        self.map[50:500, 50:50+t] = 1
        self.map[0:450, 100:100+t] = 1
        self.map[50:500, 150:150+t] = 1
        self.map[0:450, 200:200+t] = 1
        self.map[50:500, 250:250+t] = 1
        self.map[0:450, 300:300+t] = 1
        self.map[50:500, 350:350+t] = 1
        self.map[0:450, 400:400+t] = 1
        self.map[50:500, 450:450+t] = 1

        # 中心障碍物
        self.map[400:400+t, 0:40] = 1
        self.map[300:300+t, 20:50] = 1
    def get_matrix(self):
        return self.map