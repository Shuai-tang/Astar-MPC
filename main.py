from env.map_2 import Map
from astar import Astar
from astar.improverd_astar import Astar as Astar_improved
from plt import plot_map_path, plot_map_paths
from mpc import MPCController
import numpy as np

# 传统A*全局规划
def main():
    start = (55, 5)
    goal = (55, 55)
    m = Map()
    planner = Astar()
    path = planner.plan(m.map, start, goal)
    plot_map_path(m, path=path, start=start, goal=goal)

# 改进A*全局规划
def main2():
    start = (55, 5)
    goal = (55, 55)
    m = Map()
    planner = Astar_improved()
    path = planner.plan(m.map, start, goal)
    plot_map_path(m, path=path, start=start, goal=goal)

# A*全局规划 + MPC局部规划
def main3():
    start = (55, 5)
    goal = (55, 55)
    m = Map()
    planner = Astar_improved()
    global_path = planner.plan(m.map, start, goal)
    mpc = MPCController(
        N=20,
        dt=0.1,
        safe_dist=0.5,
    )

    # 从地图中提取障碍物坐标 (row, col) 对应 (x, y)
    obstacles = [tuple(p) for p in np.argwhere(m.map == 1)]

    ref_path = np.array(global_path, dtype=float)

    if len(ref_path) > 1:
        dx = ref_path[1, 0] - ref_path[0, 0]
        dy = ref_path[1, 1] - ref_path[0, 1]
        initial_theta = np.arctan2(dy, dx)
    else:
        initial_theta = 0.0

    state = np.array([float(start[0]), float(start[1]), initial_theta])

    mpc_trajectory = [tuple(state[:2])]
    path_index = 0

    max_iterations = 1000
    goal_threshold = 1.5  # 到达目标的距离阈值
    observation_radius = 10.0  # 观测范围（格子单位），仅此范围内的障碍物作为约束

    prev_state = state.copy()
    stuck_counter = 0

    for iteration in range(max_iterations):
        # 检查是否到达目标
        dist_to_goal = np.sqrt((state[0] - goal[0])**2 + (state[1] - goal[1])**2)
        if dist_to_goal < goal_threshold:
            print(f"到达目标！迭代次数: {iteration}")
            break

        # 仅将观测范围内的障碍物作为约束
        current_pos = state[:2]
        dist_to_obs = np.sqrt(np.sum((np.array(obstacles) - current_pos) ** 2, axis=1))
        obstacles_in_view = [obstacles[i] for i in range(len(obstacles)) if dist_to_obs[i] <= observation_radius]

        # 参考轨迹取“当前前方”的路径段，否则 MPC 会一直跟踪起点附近导致 v≈0 卡住
        ref_path_2d = ref_path[:, :2]
        distances = np.sum((ref_path_2d - current_pos) ** 2, axis=1)
        path_index = np.argmin(distances)
        look_ahead = min(5, len(ref_path) - path_index - 1)
        path_index = min(path_index + look_ahead, len(ref_path) - 1)
        path_segment = ref_path[path_index:]  # 从当前最近点起的一段路径
        if len(path_segment) < 2:
            path_segment = ref_path[-2:]  # 接近终点时用最后两点

        # MPC求解：传入前方路径段，使参考轨迹在机器人前方
        try:
            control = mpc.solve(state, path_segment, obstacles_in_view)
            v, omega = control
        except RuntimeError as e:
            if "Maximum_Iterations_Exceeded" in str(e) or "Infeasible" in str(e):
                # 求解未收敛时用简单前向跟踪作为回退
                next_point = path_segment[min(1, len(path_segment) - 1)]
                dx = next_point[0] - state[0]
                dy = next_point[1] - state[1]
                desired_theta = np.arctan2(dy, dx)
                theta_error = np.arctan2(np.sin(desired_theta - state[2]), np.cos(desired_theta - state[2]))
                v = 0.5
                omega = np.clip(theta_error * 2, -1.0, 1.0)
                if iteration % 50 == 0:
                    print(f"迭代 {iteration}: MPC 未收敛，使用回退控制 v={v:.2f}, omega={omega:.2f}")
            else:
                raise

        # 检查是否卡住（位置几乎不变）
        state_change = np.sqrt((state[0] - prev_state[0])**2 + (state[1] - prev_state[1])**2)
        if state_change < 0.01 and abs(v) < 0.01:
            stuck_counter += 1
            if stuck_counter > 10:
                print(f"警告：机器人可能卡住，迭代 {iteration}")
                # 强制给予一个向前的速度来恢复
                ref_path_2d = ref_path[:, :2]
                distances = np.sum((ref_path_2d - state[:2]) ** 2, axis=1)
                path_index = min(np.argmin(distances) + 1, len(ref_path) - 1)
                if path_index < len(ref_path) - 1:
                    next_point = ref_path[path_index + 1]
                    dx = next_point[0] - state[0]
                    dy = next_point[1] - state[1]
                    desired_theta = np.arctan2(dy, dx)
                    theta_error = desired_theta - state[2]
                    theta_error = np.arctan2(np.sin(theta_error), np.cos(theta_error))

                    v = 0.5
                    omega = np.clip(theta_error * 2, -1.5, 1.5)

                    print(f"  -> 强制恢复: v={v:.2f}, omega={omega:.2f}")
                else:
                    break
        else:
            stuck_counter = 0

        prev_state = state.copy()

        # 应用控制量更新状态（与 MPC 内动力学一致）
        dt = mpc.dt
        state = np.array([
            state[0] + v * np.cos(state[2]) * dt,
            state[1] + v * np.sin(state[2]) * dt,
            state[2] + omega * dt
        ])

        # 记录轨迹
        mpc_trajectory.append(tuple(state[:2]))

        # 打印进度（每50次迭代）
        if iteration % 50 == 0:
            print(f"迭代 {iteration}: 位置 ({state[0]:.2f}, {state[1]:.2f}), "
                  f"距离目标 {dist_to_goal:.2f}, 控制量 v={v:.2f}, omega={omega:.2f}")

    if iteration >= max_iterations - 1:
        print(f"达到最大迭代次数: {max_iterations}")

    print(f"MPC局部规划完成，实际轨迹长度: {len(mpc_trajectory)} 个点")

    # ========== 可视化 ==========
    print("正在可视化结果...")
    plot_map_paths(
        map_data=m,
        global_path=global_path,
        mpc_path=mpc_trajectory,
        start=start,
        goal=goal,
        show_grid=True
    )


if __name__ == "__main__":
    main()
    main2()
    main3()