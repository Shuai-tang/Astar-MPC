import casadi as ca
import numpy as np

class MPCController:
    def __init__(self, N=20, dt=0.1, safe_dist=0):
        self.N = N
        self.dt = dt
        self.safe_dist = safe_dist
        self.Q = np.diag([10.0, 10.0, 1.0])  # [x, y, theta]
        self.R = np.diag([0.1, 0.1])         # [v, omega]
        # 状态约束(v, omega)
        self.v_range = [0.0, 1.0]            # 线速度
        self.w_range = [-1.0, 1.0]           # 角速度

    def solve(self, current_state, a_star_path, obstacles):
        ref_traj = self._build_ref_path(a_star_path)
        opti = ca.Opti()
        X = opti.variable(3, self.N + 1)
        U = opti.variable(2, self.N)
        cost = self._build_cost_fn(X, U, ref_traj)
        opti.minimize(cost)
        self._build_dynamics(opti, X, U)
        self._build_constraints(opti, U, X, obstacles)
        opti.subject_to(X[:, 0] == current_state)
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 5000,           # 提高迭代上限，避免 Maximum_Iterations_Exceeded
            'ipopt.acceptable_tol': 1e-4,     # 达到可接受解即返回，不必完全收敛
            'ipopt.acceptable_iter': 50,
        }
        opti.solver('ipopt', opts)
        sol = opti.solve()
        return sol.value(U[:, 0])

    def _build_ref_path(self, path):
        ref = np.zeros((3, self.N + 1))
        for i in range(min(len(path) - 1, self.N)):
            ref[0, i] = path[i][0]
            ref[1, i] = path[i][1]
            ref[2, i] = np.arctan2(path[i+1][1] - path[i][1], 
                                   path[i+1][0] - path[i][0])
        ref[:, -1] = ref[:, -2] if self.N > 0 else ref[:, 0]
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
            for i in range(self.N + 1):
                dist_sq = (X[0, i] - obs[0])**2 + (X[1, i] - obs[1])**2
                opti.subject_to(dist_sq >= self.safe_dist**2)

    def _build_cost_fn(self, X, U, ref_traj):
        cost = 0
        for i in range(self.N):
            state_error = X[:, i] - ref_traj[:, i]
            # CasADi 无 quad_form，用 x'*Q*x = mtimes([x.T, Q, x])
            cost += ca.mtimes(ca.mtimes(state_error.T, self.Q), state_error)
            cost += ca.mtimes(ca.mtimes(U[:, i].T, self.R), U[:, i])
        return cost