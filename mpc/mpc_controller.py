import casadi as ca
import numpy as np
from scipy import ndimage
from typing import List, Tuple

class MPCController:
    def __init__(self, N=20, dt=0.1, safe_dist=1):
        self.N = N
        self.dt = dt
        self.safe_dist = safe_dist
        self.Q = np.diag([10.0, 10.0, 1.0])  # 代价函数跟踪权重 [x, y, theta]
        self.R = np.diag([0.1, 0.1])         # [v, omega]
        self.v_range = [-1.0, 3.0]            # 线速度范围
        self.w_range = [-1.0, 1.0]           # 角速度范围

    # 问题求解器
    def solve(self, current_state, a_star_path, obstacles, debug=False):
        # 将全局路径转化为参考轨迹
        ref_traj = self._build_ref_path(a_star_path)  # 参考轨迹
        opti = ca.Opti() # 构建优化问题
        X = opti.variable(3, self.N + 1) # 状态变量
        U = opti.variable(2, self.N) # 控制变量
        cost = self._build_cost_fn(X, U, ref_traj) # 构建代价函数
        opti.minimize(cost)
        self._build_dynamics(opti, X, U)
        self._build_constraints(opti, U, X, obstacles)
        opti.subject_to(X[:, 0] == current_state)
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 5000,
            'ipopt.acceptable_tol': 1e-3,
            'ipopt.acceptable_iter': 50,
            'ipopt.tol': 1e-3,
        }
        opti.solver('ipopt', opts)
        sol = opti.solve()
        return sol.value(U[:, 0])

    # 将全局路径转化为参考轨迹，全局路径格式是[[x, y, yaw], ...]（平滑后的路径）
    def _build_ref_path(self, path):

        return path

    # 构建动力学约束
    def _build_dynamics(self, opti, X, U):
        for i in range(self.N):
            theta = X[2, i]
            v = U[0, i]
            omega = U[1, i]
            next_x = X[0, i] + v * ca.cos(theta) * self.dt
            next_y = X[1, i] + v * ca.sin(theta) * self.dt
            next_theta = X[2, i] + omega * self.dt
            opti.subject_to(X[0, i+1] == next_x)
            opti.subject_to(X[1, i+1] == next_y)
            opti.subject_to(X[2, i+1] == next_theta)

    # 构建约束
    def _build_constraints(self, opti, U, X, obstacles):
        opti.subject_to(opti.bounded(self.v_range[0], U[0, :], self.v_range[1]))
        opti.subject_to(opti.bounded(self.w_range[0], U[1, :], self.w_range[1]))
        
        for obs in obstacles:
            if len(obs) >= 3:
                center_x, center_y, radius = obs[0], obs[1], obs[2]
                constraint_horizon = min(10, self.N + 1)
                for i in range(constraint_horizon):
                    dist_sq = (X[0, i] - center_x)**2 + (X[1, i] - center_y)**2
                    opti.subject_to(dist_sq >= (radius + self.safe_dist)**2)
            else:
                constraint_horizon = min(10, self.N + 1)
                for i in range(constraint_horizon):
                    dist_sq = (X[0, i] - obs[0])**2 + (X[1, i] - obs[1])**2
                    opti.subject_to(dist_sq >= self.safe_dist**2)

    # 构建障碍物预处理
    def _build_obstacle_preprocessing(self, map_matrix):
        map_matrix = np.asarray(map_matrix, dtype=int)
        h, w = map_matrix.shape
        obstacle_mask = (map_matrix == 1)
        labeled_array, num_features = ndimage.label(obstacle_mask, structure=np.ones((3, 3))) 
        obstacles = []
        for label_id in range(1, num_features + 1):
            component_mask = (labeled_array == label_id)
            component_points = np.argwhere(component_mask)  
            if len(component_points) == 0:
                continue
            center_y, center_x = component_points.mean(axis=0)
            distances = np.sqrt(
                (component_points[:, 0] - center_y)**2 + 
                (component_points[:, 1] - center_x)**2
            )
            max_radius = float(np.max(distances))
            max_radius += 0.1
            obstacles.append((float(center_x), float(center_y), max_radius))
        return obstacles

    # 构建成本函数
    def _build_cost_fn(self, X, U, ref_traj):
        cost = 0
        for i in range(self.N):
            state_error = X[:, i] - ref_traj[:, i]
            # CasADi 无 quad_form，用 x'*Q*x = mtimes([x.T, Q, x])
            cost += ca.mtimes(ca.mtimes(state_error.T, self.Q), state_error)
            cost += ca.mtimes(ca.mtimes(U[:, i].T, self.R), U[:, i])
        return cost