# Astar-MPC

基于 **A\*** 与 **MPC** 的路径规划项目：A\* 负责全局路径规划，MPC 负责局部路径规划。

## 概述

- **A\***：在已知栅格地图上搜索从起点到终点的全局最优/可行路径，得到一条粗粒度参考轨迹。
- **MPC（模型预测控制）**：在全局路径附近进行局部轨迹优化，考虑动力学约束与障碍物，实现平滑、可跟踪的局部路径。

## 效果对比

<table>
<tr>
<td width="50%">

**传统 A* 算法**

![传统Astar](media/传统Astar.png)

</td>
<td width="50%">

**改进 A* 算法**

![改进Astar](media/改进Astar.png)

</td>
</tr>
</table>

## 项目结构

```
Astar-MPC/
├── main.py          # 程序入口，串联全局规划与局部规划
├── README.md
├── env/             # 环境与地图
│   └── map.py       # 栅格地图类 Map，定义障碍物、起点、终点
├── astar/           # A* 全局路径规划模块
├── mpc/             # MPC 局部路径规划模块
└── plt/             # 绘图与可视化
```

### 参考资料

- [A*算法（A-star Algorithm）详解：理论与实例](https://zhuanlan.zhihu.com/p/590786438)
- [路径规划之 A* 算法](https://zhuanlan.zhihu.com/p/54510444)
- [基于改进A星算法融合改进动态窗口法的无人机动态避障方法研究_朱亚凯.pdf](paper_ref/基于改进A星算法融合改进动态窗口法的无人机动态避障方法研究_朱亚凯.pdf)