import torch
import matplotlib.pyplot as plt

# --- 1. 配置与超参数 ---
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
K = 1000  # 采样轨迹数 (Samples)
T = 30  # 预测时域 (Horizon)
DT = 0.1  # 步长
LAMBDA = 0.05  # 温度参数 (控制对低代价路径的敏感度)
ITERATIONS = 100  # 仿真总步数

# 控制量约束 (速度, 角速度)
U_MIN = torch.tensor([0.0, -2.0], device=DEVICE)
U_MAX = torch.tensor([2.0, 2.0], device=DEVICE)
SIGMA = torch.tensor([0.5, 0.5], device=DEVICE)  # 采样噪声标准差

# 障碍物位置 [x, y, radius]
OBSTACLE = torch.tensor([2.5, 2.5, 0.8], device=DEVICE)
GOAL = torch.tensor([5.0, 5.0], device=DEVICE)


# --- 2. 动力学模型 (支持 Batch) ---
def dynamics(state, action):
    """
    状态 x: [x, y, theta]
    控制 u: [v, omega]
    """
    x, y, theta = state[:, 0], state[:, 1], state[:, 2]
    v, omega = action[:, 0], action[:, 1]

    new_x = x + v * torch.cos(theta) * DT
    new_y = y + v * torch.sin(theta) * DT
    new_theta = theta + omega * DT

    return torch.stack([new_x, new_y, new_theta], dim=1)


# --- 3. 代价函数 ---
def compute_cost(state, action, goal, obs):
    # 距离目标的代价 (越近越小)
    dist_to_goal = torch.norm(state[:, :2] - goal, dim=1)

    # 碰撞代价 (靠近障碍物时代价剧增)
    dist_to_obs = torch.norm(state[:, :2] - obs[:2], dim=1)
    collision_cost = torch.where(dist_to_obs < obs[2], 1000.0, 0.0)

    # 边界代价 (简单约束)
    boundary_cost = torch.where((state[:, 0].abs() > 10) | (state[:, 1].abs() > 10), 500.0, 0.0)

    return dist_to_goal + collision_cost + boundary_cost


# --- 4. MPPI 控制器 ---
class MPPI:
    def __init__(self):
        self.U = torch.zeros((T, 2), device=DEVICE)  # 初始控制序列 [T, control_dim]

    def control(self, state_0):
        # 准备噪声 epsilon: (K, T, 2)
        epsilon = torch.randn((K, T, 2), device=DEVICE) * SIGMA

        # 复制初始状态 K 份: (K, 3)
        state = state_0.repeat(K, 1)
        total_costs = torch.zeros(K, device=DEVICE)

        # Rollout: 并行预测 T 步
        for t in range(T):
            # 这里的控制量 = 当前均值 + 噪声，并做限幅
            u_t = torch.clamp(self.U[t] + epsilon[:, t, :], U_MIN, U_MAX)
            state = dynamics(state, u_t)
            total_costs += compute_cost(state, u_t, GOAL, OBSTACLE)

        # 计算权重 (Softmax 技巧)
        beta = torch.min(total_costs)
        weights = torch.exp(-1.0 / LAMBDA * (total_costs - beta))
        weights /= torch.sum(weights)  # (K,)

        # 更新控制序列 (加权求和噪声)
        self.U = self.U + torch.sum(weights.view(K, 1, 1) * epsilon, dim=0)

        # 取得当前动作并平移序列 (Hotstart)
        action = self.U[0].clone()
        self.U = torch.roll(self.U, -1, dims=0)
        self.U[-1] = torch.tensor([0.0, 0.0], device=DEVICE)

        return action


# --- 5. 仿真循环 ---
controller = MPPI()
curr_state = torch.tensor([0.0, 0.0, 0.0], device=DEVICE)  # 起点
history = [curr_state[:2].cpu().numpy()]

print("仿真开始...")
for i in range(ITERATIONS):
    action = controller.control(curr_state)
    curr_state = dynamics(curr_state.unsqueeze(0), action.unsqueeze(0)).squeeze(0)
    history.append(curr_state[:2].cpu().numpy())

    if torch.norm(curr_state[:2] - GOAL) < 0.2:
        print(f"到达目标点！步数: {i}")
        break

# --- 6. 可视化 ---
history = torch.tensor(history)
plt.figure(figsize=(8, 8))
plt.plot(history[:, 0], history[:, 1], '-o', label='Path', markersize=2)
plt.scatter(GOAL[0].cpu(), GOAL[1].cpu(), c='g', marker='*', s=200, label='Goal')
circle = plt.Circle(OBSTACLE[:2].cpu(), OBSTACLE[2].cpu(), color='r', alpha=0.5, label='Obstacle')
plt.gca().add_patch(circle)
plt.xlim(-1, 7);
plt.ylim(-1, 7);
plt.legend();
plt.grid(True)
plt.title("MPPI Path Tracking with Obstacle Avoidance")
plt.show()