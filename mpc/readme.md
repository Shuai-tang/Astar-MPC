# MPC 模块说明

基于 CasADi + IPOPT 的差分轮/独轮车模型 MPC 控制器，用于在 A* 全局路径上进行局部跟踪与避障。

---

## MPCController 类

### 初始化 `__init__(N=20, dt=0.1, safe_dist=0)`

| 参数       | 含义           | 默认值 |
|------------|----------------|--------|
| `N`        | 预测时域步数   | 20     |
| `dt`       | 离散化时间步长 | 0.1    |
| `safe_dist`| 与障碍物的最小距离约束（格子单位） | 0 |

内部固定参数：
- 状态权重 `Q = diag([10, 10, 1])`（x, y, θ）
- 控制权重 `R = diag([0.1, 0.1])`（v, ω）
- 线速度约束 `v_range = [0, 1]`
- 角速度约束 `w_range = [-1, 1]`

---

### 求解接口 `solve(current_state, a_star_path, obstacles)`

- **current_state**：`(3,)` 数组 `[x, y, theta]`（与地图格子坐标一致）
- **a_star_path**：全局路径，形状 `(n, 2)` 或可索引为 `path[i][0]`, `path[i][1]` 的序列
- **obstacles**：障碍物列表，每个元素为 `(x, y)` 或 `[x, y]`（建议只传入**观测范围内**的障碍物）

**返回**：当前步最优控制 `(v, omega)`，即 `U[:, 0]`。求解失败时由 IPOPT 抛异常。

---

## 内部构建函数（供类内调用）

### `_build_ref_path(path)`

从 A* 路径点（仅坐标）构建参考轨迹 `ref`，形状 `(3, N+1)`：
- `ref[0:2, i]`：第 i 点的 x, y
- `ref[2, i]`：由相邻点方向得到的参考航向角 `atan2(dy, dx)`

### `_build_dynamics(opti, X, U)`

离散化独轮车动力学（欧拉一步）：
- `x_{k+1} = x_k + v*cos(θ)*dt`
- `y_{k+1} = y_k + v*sin(θ)*dt`
- `θ_{k+1} = θ_k + omega*dt`

### `_build_constraints(opti, U, X, obstacles)`

- 控制量：`v`、`omega` 的上下界
- 障碍物：对每个障碍点、每个预测步，约束 `(x-x_obs)^2 + (y-y_obs)^2 >= safe_dist^2`

### `_build_cost_fn(X, U, ref_traj)`

- 状态误差：`(x - ref)^T Q (x - ref)`（用 CasADi `mtimes` 实现二次型）
- 控制代价：`u^T R u`
- 对预测时域内每一步求和

---

## 在 main 中的使用方式

1. **障碍物**：从地图 `m.map == 1` 得到全部障碍坐标，再按**观测范围**过滤：只把与当前机器人位置距离 ≤ `observation_radius` 的障碍物传入 `solve`，即仅观测范围内的障碍物作为约束。
2. **状态更新**：控制器不提供 `simulate`，在 main 中按相同动力学用 `mpc.dt` 和当前控制量 `(v, omega)` 更新 `state`。
3. **参考路径**：每次 `solve` 传入完整 A* 路径 `ref_path`，内部通过 `_build_ref_path` 生成参考轨迹。

---

## 依赖

- `casadi`
- `numpy`
- IPOPT（通过 CasADi 调用）
