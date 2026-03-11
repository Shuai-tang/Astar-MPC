"""
MPC 局部路径跟踪控制器
基于运动学模型，对参考路径进行跟踪优化
"""
import numpy as np
from typing import List, Tuple, Optional
from scipy.optimize import minimize


class MPCController:
    """
    标准 MPC 控制器，使用差速/单车运动学模型
    状态: [x, y, theta]
    控制: [v, omega]
    """

    def __init__(
        self,
        horizon: int = 10,
        dt: float = 0.1,
        v_max: float = 2.0,
        v_min: float = -0.5,
        omega_max: float = 1.5,
        q_pos: float = 1.0,
        q_theta: float = 0.5,
        r_v: float = 0.1,
        r_omega: float = 0.1,
    ):
        """
        Args:
            horizon: 预测步数 N
            dt: 离散时间步长
            v_max, v_min: 线速度上下界
            omega_max: 角速度上界 (对称)
            q_pos: 位置误差权重
            q_theta: 航向误差权重
            r_v, r_omega: 控制量惩罚权重
        """
        self.N = horizon
        self.dt = dt
        self.v_max = v_max
        self.v_min = v_min
        self.omega_max = omega_max
        self.q_pos = q_pos
        self.q_theta = q_theta
        self.r_v = r_v
        self.r_omega = r_omega
        self.nx = 3  # x, y, theta
        self.nu = 2  # v, omega

    def solve(
        self,
        state: np.ndarray,
        ref_path: np.ndarray,
        path_index: int = 0,
    ) -> Optional[Tuple[float, float]]:
        """
        求解 MPC，返回当前步的最优控制

        Args:
            state: 当前状态 [x, y, theta]
            ref_path: 参考路径 Nx2 或 Nx3，每行 (x, y) 或 (x, y, theta)
            path_index: 当前在参考路径上的索引，用于截取未来 N 步参考

        Returns:
            (v, omega) 最优控制，无解时返回 None
        """
        state = np.asarray(state, dtype=float).ravel()
        ref_path = np.asarray(ref_path, dtype=float)
        if ref_path.ndim == 1:
            ref_path = ref_path.reshape(-1, 2)

        # 提取未来 N 步参考
        ref_slice = ref_path[path_index : path_index + self.N]
        if len(ref_slice) == 0:
            return None

        # 不足 N 步时用最后一点填充
        if len(ref_slice) < self.N:
            last = ref_slice[-1]
            ref_slice = np.vstack([ref_slice, np.tile(last, (self.N - len(ref_slice), 1))])

        # 构建参考轨迹 [x_ref, y_ref, theta_ref] x N
        ref_traj = self._build_ref_traj(ref_slice)

        # 决策变量: [v_0, omega_0, ..., v_{N-1}, omega_{N-1}]
        n_opt = self.N * self.nu
        x0 = np.zeros(n_opt)
        bounds = []
        for _ in range(self.N):
            bounds.append((self.v_min, self.v_max))
            bounds.append((-self.omega_max, self.omega_max))

        def cost(u_flat: np.ndarray) -> float:
            return self._cost_fn(state, u_flat, ref_traj)

        res = minimize(cost, x0, method="L-BFGS-B", bounds=bounds)
        if not res.success:
            return None

        u_opt = res.x
        v, omega = u_opt[0], u_opt[1]
        return (float(v), float(omega))

    def _build_ref_traj(self, ref_slice: np.ndarray) -> np.ndarray:
        """构建 Nx3 参考轨迹，补全 theta"""
        n = len(ref_slice)
        traj = np.zeros((n, 3))
        traj[:, :2] = ref_slice[:, :2]
        for i in range(1, n):
            dx = ref_slice[i, 0] - ref_slice[i - 1, 0]
            dy = ref_slice[i, 1] - ref_slice[i - 1, 1]
            traj[i, 2] = np.arctan2(dy, dx)
        traj[0, 2] = traj[1, 2] if n > 1 else 0
        if ref_slice.shape[1] >= 3:
            traj[:, 2] = ref_slice[:, 2]
        return traj

    def _cost_fn(self, state: np.ndarray, u_flat: np.ndarray, ref_traj: np.ndarray) -> float:
        """MPC 代价函数"""
        x = state.copy()
        cost_val = 0.0

        for k in range(self.N):
            v = u_flat[k * 2]
            omega = u_flat[k * 2 + 1]
            ref = ref_traj[k]

            # 状态代价
            cost_val += self.q_pos * ((x[0] - ref[0]) ** 2 + (x[1] - ref[1]) ** 2)
            dtheta = x[2] - ref[2]
            dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))
            cost_val += self.q_theta * (dtheta**2)

            # 控制代价
            cost_val += self.r_v * (v**2) + self.r_omega * (omega**2)

            # 前向仿真
            x = self._step(x, v, omega)

        return cost_val

    def _step(self, x: np.ndarray, v: float, omega: float) -> np.ndarray:
        """运动学一步: x' = v*cos(θ), y' = v*sin(θ), θ' = omega"""
        x_new = x.copy()
        x_new[0] += v * np.cos(x[2]) * self.dt
        x_new[1] += v * np.sin(x[2]) * self.dt
        x_new[2] += omega * self.dt
        x_new[2] = np.arctan2(np.sin(x_new[2]), np.cos(x_new[2]))
        return x_new

    def simulate(
        self,
        state: np.ndarray,
        u: Tuple[float, float],
    ) -> np.ndarray:
        """
        单步仿真，用于主循环中更新状态

        Args:
            state: [x, y, theta]
            u: (v, omega)

        Returns:
            下一步状态 [x, y, theta]
        """
        return self._step(np.asarray(state), u[0], u[1])
