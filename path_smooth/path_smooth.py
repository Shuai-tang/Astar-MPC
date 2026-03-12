import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d


class PathSmoother:
    """基于三次样条与可选高斯滤波的 A* 路径平滑器。"""

    def __init__(self, step_size=0.5, smoothing_factor=1.0, use_gaussian=True):
        """
        Args:
            step_size: 重采样步长
            smoothing_factor: 高斯滤波的 sigma
            use_gaussian: 是否使用高斯预平滑与后平滑
        """
        self.step_size = step_size
        self.smoothing_factor = smoothing_factor
        self.use_gaussian = use_gaussian

    def smooth(self, path, step_size=None, smoothing_factor=None, use_gaussian=None):
        """
        对 A* 路径进行平滑，返回 x, y, yaw 数组。

        Args:
            path: 路径点列表/数组，形状 (N, 2)
            step_size: 重采样步长，为 None 时使用实例默认值
            smoothing_factor: 高斯 sigma，为 None 时使用实例默认值
            use_gaussian: 是否使用高斯滤波，为 None 时使用实例默认值

        Returns:
            x_smooth, y_smooth, yaw_smooth: 平滑后的 x、y 与航向角数组
        """
        step_size = step_size if step_size is not None else self.step_size
        smoothing_factor = smoothing_factor if smoothing_factor is not None else self.smoothing_factor
        use_gaussian = use_gaussian if use_gaussian is not None else self.use_gaussian

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
        ds[ds < 1e-6] = 1e-6
        s = np.zeros(len(x))
        s[1:] = np.cumsum(ds)

        if s[-1] < 1e-6:
            return x, y, np.zeros(len(x))

        # 2. 建立三次样条插值函数
        try:
            cs_x = CubicSpline(s, x, bc_type='natural', extrapolate=False)
            cs_y = CubicSpline(s, y, bc_type='natural', extrapolate=False)
        except Exception:
            return x, y, np.zeros(len(x))

        # 3. 在 0 到总弧长之间进行重采样
        s_new = np.arange(0, s[-1] + step_size / 2, step_size)
        s_new = np.clip(s_new, 0, s[-1])
        s_new = np.unique(s_new)

        x_smooth = cs_x(s_new)
        y_smooth = cs_y(s_new)

        # 4. 后处理：再次轻微高斯滤波
        if use_gaussian and len(x_smooth) > 3:
            x_smooth = gaussian_filter1d(x_smooth, sigma=0.5)
            y_smooth = gaussian_filter1d(y_smooth, sigma=0.5)

        # 5. 计算航向角
        dx_ds = np.gradient(x_smooth, s_new)
        dy_ds = np.gradient(y_smooth, s_new)
        dx_ds[dx_ds == 0] = 1e-6
        yaw_smooth = np.arctan2(dy_ds, dx_ds)

        return x_smooth, y_smooth, yaw_smooth

