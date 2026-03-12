from env.map_3 import Map
from astar.improverd_astar import Astar as Astar_improved
from path_smooth.path_smooth import PathSmoother
from plt.plot_map_path import plot_three_paths
from mpc import MPCController
import numpy as np

def extract_obstacles_in_range(obstacles_list, center_pos, radius=5.0):
    """
    从预处理后的障碍物列表中提取观测范围内的障碍物
    Args:
        obstacles_list: 预处理后的障碍物列表，格式为 [(center_x, center_y, radius), ...]
        center_pos: 观测中心位置 (x, y)
        radius: 观测半径（栅格数）
    Returns:
        nearby_obstacles: 观测范围内的障碍物列表
    """
    nearby_obstacles = []
    center_x, center_y = center_pos[0], center_pos[1]
    
    for obs in obstacles_list:
        obs_x, obs_y, obs_radius = obs[0], obs[1], obs[2]
        # 计算障碍物中心到观测中心的距离
        dist = np.sqrt((obs_x - center_x)**2 + (obs_y - center_y)**2)
        # 如果障碍物在观测范围内（考虑障碍物半径）
        if dist <= radius + obs_radius:
            nearby_obstacles.append(obs)
    
    return nearby_obstacles

def main():
    # ========== 地图与起终点 ==========
    start = (2, 18)
    goal = (18, 2)
    m = Map()
    
    # ========== 全局路径规划：A* ==========
    print("正在进行全局路径规划（A*）...")
    planner = Astar_improved()
    astar_path = planner.plan(m.map, start, goal)
    
    if astar_path is None:
        print("错误：无法找到全局路径！")
        return
    
    print(f"A*路径规划完成，路径长度: {len(astar_path)} 个点")
    
    # ========== 轨迹平滑 ==========
    print("正在进行轨迹平滑...")
    smoother = PathSmoother(step_size=0.5, smoothing_factor=1.0, use_gaussian=True)
    x_smooth, y_smooth, yaw_smooth = smoother.smooth(astar_path)
    
    # 平滑路径格式：[(x, y, yaw), ...]
    smooth_path = [(float(x), float(y), float(yaw)) for x, y, yaw in zip(x_smooth, y_smooth, yaw_smooth)]
    smooth_path_coords = [(p[0], p[1]) for p in smooth_path]  # 用于绘图的坐标
    
    print(f"轨迹平滑完成，平滑后路径长度: {len(smooth_path)} 个点")
    
    # ========== 初始化MPC控制器 ==========
    print("正在初始化MPC控制器...")
    mpc = MPCController(
        N=15,           # 预测步数
        dt=0.1,         # 时间步长
        safe_dist=0.3   # 安全距离
    )
    
    # ========== 障碍物预处理 ==========
    print("正在进行障碍物预处理...")
    all_obstacles = mpc._build_obstacle_preprocessing(m.map)
    print(f"预处理完成，识别到 {len(all_obstacles)} 个障碍物区域")
    
    # 观测范围半径（栅格数）
    observation_radius = 5.0
    
    # ========== 初始化机器人状态 ==========
    # 计算初始航向角（从起点指向路径上第一个点）
    if len(smooth_path) > 1:
        dx = smooth_path[1][0] - smooth_path[0][0]
        dy = smooth_path[1][1] - smooth_path[0][1]
        initial_yaw = np.arctan2(dy, dx)
    else:
        initial_yaw = 0.0
    
    initial_state = np.array([float(start[0]), float(start[1]), initial_yaw])
    
    # 检查起点是否在障碍物内或太靠近障碍物
    start_obstacles = extract_obstacles_in_range(
        all_obstacles, 
        (initial_state[0], initial_state[1]), 
        radius=observation_radius
    )
    start_too_close = False
    for obs in start_obstacles:
        obs_x, obs_y, obs_radius = obs[0], obs[1], obs[2]
        dist = np.sqrt((initial_state[0] - obs_x)**2 + (initial_state[1] - obs_y)**2)
        if dist < obs_radius + mpc.safe_dist:
            start_too_close = True
            print(f"警告：起点太靠近障碍物 (距离={dist:.2f} < 障碍物半径+安全距离={obs_radius + mpc.safe_dist:.2f})")
            break
    
    if start_too_close:
        print("建议：减小安全距离或调整起点位置")
    
    # ========== MPC局部路径规划 ==========
    print("正在进行局部路径规划（MPC）...")
    
    mpc_trajectory = [(initial_state[0], initial_state[1])]
    current_state = initial_state.copy()
    prev_state = initial_state.copy()
    stuck_counter = 0  # 卡住计数器
    
    # MPC控制循环
    max_iterations = 200
    goal_threshold = 1.0  # 到达目标的距离阈值
    
    for iteration in range(max_iterations):
        # 检查是否到达目标
        dist_to_goal = np.sqrt((current_state[0] - goal[0])**2 + (current_state[1] - goal[1])**2)
        if dist_to_goal < goal_threshold:
            print(f"到达目标！迭代次数: {iteration}")
            break
        
        # 提取观测范围内的障碍物（使用预处理后的圆形障碍物）
        nearby_obstacles = extract_obstacles_in_range(
            all_obstacles, 
            (current_state[0], current_state[1]), 
            radius=observation_radius
        )
        
        # 找到当前状态在平滑路径上的最近点索引
        if len(smooth_path) > 0:
            distances = [np.sqrt((current_state[0] - p[0])**2 + (current_state[1] - p[1])**2) 
                         for p in smooth_path]
            path_index = np.argmin(distances)
            
            # 向前看：选择路径上更靠前的点作为参考
            look_ahead = min(5, len(smooth_path) - path_index - 1)
            path_index = min(path_index + look_ahead, len(smooth_path) - 1)
            
            # 提取当前参考路径段
            ref_path_segment = smooth_path[path_index:path_index + mpc.N + 1]
            if len(ref_path_segment) < mpc.N + 1:
                # 不足时用最后一个点填充
                last_point = smooth_path[-1]
                ref_path_segment = ref_path_segment + [last_point] * (mpc.N + 1 - len(ref_path_segment))
            
            # 修正参考路径：如果路径点太靠近障碍物，将其向外推离
            corrected_ref_path = []
            for ref_point in ref_path_segment:
                ref_x, ref_y, ref_yaw = ref_point[0], ref_point[1], ref_point[2]
                corrected_x, corrected_y = ref_x, ref_y
                
                # 检查是否太靠近障碍物
                for obs in nearby_obstacles:
                    if len(obs) >= 3:
                        obs_x, obs_y, obs_r = obs[0], obs[1], obs[2]
                        dist = np.sqrt((ref_x - obs_x)**2 + (ref_y - obs_y)**2)
                        min_required_dist = obs_r + mpc.safe_dist
                        
                        if dist < min_required_dist:
                            # 将参考点向外推离障碍物
                            if dist > 0.01:  # 避免除零
                                direction_x = (ref_x - obs_x) / dist
                                direction_y = (ref_y - obs_y) / dist
                            else:
                                # 如果点在障碍物中心，使用航向角方向
                                direction_x = np.cos(ref_yaw)
                                direction_y = np.sin(ref_yaw)
                            
                            corrected_x = obs_x + direction_x * min_required_dist
                            corrected_y = obs_y + direction_y * min_required_dist
                            break  # 只修正第一个冲突的障碍物
                
                corrected_ref_path.append((corrected_x, corrected_y, ref_yaw))
            
            ref_path_segment = corrected_ref_path
        else:
            break
        
        # MPC求解
        control = None
        try:
            if iteration == 0 or iteration % 20 == 0:
                print(f"迭代 {iteration}:")
                print(f"  当前位置: ({current_state[0]:.2f}, {current_state[1]:.2f})")
                print(f"  观测范围内障碍物数量: {len(nearby_obstacles)}")
                if len(nearby_obstacles) > 0:
                    print(f"  障碍物示例: {nearby_obstacles[0]}")
            
            # 在第一次迭代或之前失败时启用调试模式
            debug_mode = (iteration == 0) or (iteration < 5)
            control = mpc.solve(current_state, ref_path_segment, nearby_obstacles, debug=debug_mode)
            
        except Exception as e:
            print(f"MPC求解异常，迭代 {iteration}: {e}")
            control = None
        
        # 如果MPC求解失败，使用备用策略
        if control is None:
            # 备用策略：优先朝向目标，如果目标被阻挡则朝向参考路径
            # 计算到目标的方向
            dx_to_goal = goal[0] - current_state[0]
            dy_to_goal = goal[1] - current_state[1]
            dist_to_goal_vec = np.sqrt(dx_to_goal**2 + dy_to_goal**2)
            
            # 检查到目标的路径是否被障碍物阻挡
            goal_blocked = False
            if dist_to_goal_vec > 0.1:
                # 检查路径上是否有障碍物
                for obs in nearby_obstacles:
                    if len(obs) >= 3:
                        obs_x, obs_y, obs_r = obs[0], obs[1], obs[2]
                        # 计算障碍物到当前-目标连线的距离
                        # 使用点到线段距离公式
                        A = np.array([current_state[0], current_state[1]])
                        B = np.array([goal[0], goal[1]])
                        P = np.array([obs_x, obs_y])
                        
                        AB = B - A
                        AP = P - A
                        if np.dot(AB, AB) > 1e-6:
                            t = np.clip(np.dot(AP, AB) / np.dot(AB, AB), 0, 1)
                            closest_point = A + t * AB
                            dist_to_line = np.linalg.norm(P - closest_point)
                            if dist_to_line < obs_r + mpc.safe_dist:
                                goal_blocked = True
                                break
            
            if not goal_blocked and dist_to_goal_vec > 0.1:
                # 直接朝向目标
                desired_theta = np.arctan2(dy_to_goal, dx_to_goal)
                v = min(0.8, dist_to_goal_vec * 0.3)
            elif len(ref_path_segment) > 0:
                # 朝向参考路径
                ref_x, ref_y = ref_path_segment[0][0], ref_path_segment[0][1]
                dx = ref_x - current_state[0]
                dy = ref_y - current_state[1]
                dist_to_ref = np.sqrt(dx**2 + dy**2)
                
                if dist_to_ref > 0.1:
                    desired_theta = np.arctan2(dy, dx)
                    v = min(0.5, dist_to_ref * 0.5)
                else:
                    # 如果参考点太近，朝向目标
                    desired_theta = np.arctan2(dy_to_goal, dx_to_goal)
                    v = min(0.5, dist_to_goal_vec * 0.3)
            else:
                # 最后选择：朝向目标
                desired_theta = np.arctan2(dy_to_goal, dx_to_goal)
                v = min(0.5, dist_to_goal_vec * 0.3)
            
            # 计算角速度
            theta_error = desired_theta - current_state[2]
            theta_error = np.arctan2(np.sin(theta_error), np.cos(theta_error))
            omega = np.clip(theta_error * 2.0, -mpc.w_range[1], mpc.w_range[1])
            
            control = np.array([v, omega])
            print(f"  使用备用策略，v={v:.2f}, omega={omega:.2f}")
        
        v, omega = control[0], control[1]
        
        # 限制控制量在合理范围内
        v = np.clip(v, mpc.v_range[0], mpc.v_range[1])
        omega = np.clip(omega, mpc.w_range[0], mpc.w_range[1])
        
        # 应用控制量，更新状态（使用MPC的动力学模型）
        dt = mpc.dt
        prev_state = current_state.copy()
        current_state[0] += v * np.cos(current_state[2]) * dt
        current_state[1] += v * np.sin(current_state[2]) * dt
        current_state[2] += omega * dt
        current_state[2] = np.arctan2(np.sin(current_state[2]), np.cos(current_state[2]))  # 归一化角度
        
        # 检查是否卡住
        state_change = np.sqrt((current_state[0] - prev_state[0])**2 + 
                               (current_state[1] - prev_state[1])**2)
        if state_change < 0.01 and abs(v) < 0.01:
            stuck_counter += 1
            if stuck_counter > 10:
                print(f"  警告：机器人可能卡住，尝试绕过障碍物")
                # 尝试侧向移动
                current_state[2] += np.pi / 4  # 旋转45度
                stuck_counter = 0
        else:
            stuck_counter = 0
        
        # 记录轨迹
        mpc_trajectory.append((current_state[0], current_state[1]))
        
        # 打印进度（每20次迭代）
        if iteration % 20 == 0:
            print(f"迭代 {iteration}: 位置 ({current_state[0]:.2f}, {current_state[1]:.2f}), "
                  f"距离目标 {dist_to_goal:.2f}, 控制量 v={v:.2f}, omega={omega:.2f}")
    
    if iteration >= max_iterations - 1:
        print(f"达到最大迭代次数: {max_iterations}")
    
    print(f"MPC局部规划完成，实际轨迹长度: {len(mpc_trajectory)} 个点")
    
    # ========== 可视化 ==========
    print("正在可视化结果...")
    plot_three_paths(
        map_data=m.map,
        astar_path=astar_path,
        smooth_path=smooth_path_coords,
        mpc_path=mpc_trajectory,
        start=start,
        goal=goal,
        show_grid=True
    )
    
    # ========== 统计信息 ==========
    print("\n路径统计信息：")
    print(f"A*路径点数: {len(astar_path)}")
    print(f"平滑路径点数: {len(smooth_path)}")
    print(f"MPC轨迹点数: {len(mpc_trajectory)}")
    print(f"障碍物区域数量: {len(all_obstacles)}")

if __name__ == "__main__":
    main()
