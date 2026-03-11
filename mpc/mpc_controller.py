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

    def solve(self, current_state, a_star_path, obstacles, debug=False):
        # 将全局路径转化为参考轨迹
        ref_traj = self._build_ref_path(a_star_path)
        
        # 诊断：检查参考路径是否穿过障碍物
        if debug:
            print(f"  诊断信息：")
            print(f"    当前状态: ({current_state[0]:.2f}, {current_state[1]:.2f})")
            print(f"    参考路径点数: {len(a_star_path)}")
            print(f"    障碍物数量: {len(obstacles)}")
            
            # 检查参考路径是否在障碍物内
            ref_in_obstacle = False
            for i, ref_point in enumerate(a_star_path[:min(5, len(a_star_path))]):
                ref_x, ref_y = ref_point[0], ref_point[1]
                for obs in obstacles:
                    if len(obs) >= 3:
                        obs_x, obs_y, obs_r = obs[0], obs[1], obs[2]
                        dist = np.sqrt((ref_x - obs_x)**2 + (ref_y - obs_y)**2)
                        if dist < obs_r + self.safe_dist:
                            ref_in_obstacle = True
                            print(f"    警告：参考路径点 {i} ({ref_x:.2f}, {ref_y:.2f}) 太靠近障碍物")
                            print(f"      距离障碍物中心: {dist:.2f}, 需要: {obs_r + self.safe_dist:.2f}")
            
            # 检查当前状态是否在障碍物内
            current_in_obstacle = False
            for obs in obstacles:
                if len(obs) >= 3:
                    obs_x, obs_y, obs_r = obs[0], obs[1], obs[2]
                    dist = np.sqrt((current_state[0] - obs_x)**2 + (current_state[1] - obs_y)**2)
                    if dist < obs_r + self.safe_dist:
                        current_in_obstacle = True
                        print(f"    警告：当前状态太靠近障碍物")
                        print(f"      距离障碍物中心: {dist:.2f}, 需要: {obs_r + self.safe_dist:.2f}")
        
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
        try:
            sol = opti.solve()
            return sol.value(U[:, 0])
        except Exception as e:
            if debug:
                print(f"    求解失败原因分析:")
                print(f"      约束数量: {len(obstacles)} 个障碍物 × {min(10, self.N + 1)} 个时间步")
                print(f"      安全距离: {self.safe_dist}")
                print(f"      预测步数: {self.N}")
            return None

    # 将全局路径转化为参考轨迹，全局路径格式是[(x, y, yaw), ...]（平滑后的路径）
    def _build_ref_path(self, path):
        ref = np.zeros((3, self.N + 1))
        num_points = min(len(path), self.N + 1)
        for i in range(num_points):
            ref[0, i] = path[i][0]  # x坐标
            ref[1, i] = path[i][1]  # y坐标
            ref[2, i] = path[i][2]  # 航向角（直接使用）
        if num_points < self.N + 1:
            last_x = path[-1][0]
            last_y = path[-1][1]
            last_yaw = path[-1][2]
            for i in range(num_points, self.N + 1):
                ref[0, i] = last_x
                ref[1, i] = last_y
                ref[2, i] = last_yaw
        return ref

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

    def _build_cost_fn(self, X, U, ref_traj):
        cost = 0
        for i in range(self.N):
            state_error = X[:, i] - ref_traj[:, i]
            # CasADi 无 quad_form，用 x'*Q*x = mtimes([x.T, Q, x])
            cost += ca.mtimes(ca.mtimes(state_error.T, self.Q), state_error)
            cost += ca.mtimes(ca.mtimes(U[:, i].T, self.R), U[:, i])
        return cost