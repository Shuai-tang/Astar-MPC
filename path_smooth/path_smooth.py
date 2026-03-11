import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d

def smooth_astar_path(path, step_size=0.5, smoothing_factor=1.0, use_gaussian=True):
    if len(path) < 2:
        return np.array([path[0][0]]), np.array([path[0][1]]), np.array([0.0])
    
    path = np.array(path)
    x = path[:, 0].astype(float)
    y = path[:, 1].astype(float)

    # 0. 可选：使用高斯滤波进行预平滑，减少高频噪声
    if use_gaussian and len(path) > 3:
        x = gaussian_filter1d(x, sigma=smoothing_factor)
        y = gaussian_filter1d(y, sigma=smoothing_factor)

    # 1. 计算每个点之间的距离，生成累计弧长 s
    ds = np.hypot(np.diff(x), np.diff(y))
    # 避免除零错误：如果两个点重合，设置最小距离
    ds[ds < 1e-6] = 1e-6
    s = np.zeros(len(x))
    s[1:] = np.cumsum(ds)

    # 如果总弧长为0（所有点重合），返回原始路径
    if s[-1] < 1e-6:
        return x, y, np.zeros(len(x))

    # 2. 建立三次样条插值函数 (x 和 y 分别关于 s 的函数)
    # bc_type='natural' 表示边界处二阶导数为0，更平滑
    # 使用 ext_type='linear' 避免边界处的振荡
    try:
        cs_x = CubicSpline(s, x, bc_type='natural', extrapolate=False)
        cs_y = CubicSpline(s, y, bc_type='natural', extrapolate=False)
    except:
        # 如果样条插值失败，返回原始路径
        return x, y, np.zeros(len(x))

    # 3. 在 0 到 总弧长 之间进行重采样
    # 确保包含最后一个点
    s_new = np.arange(0, s[-1] + step_size/2, step_size)
    s_new = np.clip(s_new, 0, s[-1])  # 确保不超出范围
    s_new = np.unique(s_new)  # 去重
    
    x_smooth = cs_x(s_new)
    y_smooth = cs_y(s_new)

    # 4. 后处理：对平滑后的路径再次应用轻微的高斯滤波，进一步减少振荡
    if use_gaussian and len(x_smooth) > 3:
        x_smooth = gaussian_filter1d(x_smooth, sigma=0.5)
        y_smooth = gaussian_filter1d(y_smooth, sigma=0.5)

    # 5. 计算航向角 (Yaw)
    # 通过一阶导数 dy/dx = (dy/ds) / (dx/ds) 得到
    dx_ds = np.gradient(x_smooth, s_new)
    dy_ds = np.gradient(y_smooth, s_new)
    # 避免除零错误
    dx_ds[dx_ds == 0] = 1e-6
    yaw_smooth = np.arctan2(dy_ds, dx_ds)

    return x_smooth, y_smooth, yaw_smooth