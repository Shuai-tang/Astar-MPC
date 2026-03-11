# astar — A* 与变体路径规划

本目录包含基于 A* 的全局路径规划实现，包括标准 A* 及各类变体，用于栅格地图上的最短/可行路径搜索。

---

## 目录结构

| 文件 | 说明 |
|------|------|
| `astar_planner.py` | 标准 A* 实现（`AstarPlanner`、`Node`） |
| `__init__.py` | 包导出 |
| `README.md` | 本说明与注释文档 |

（后续可在此补充变体文件，如 `astar_weighted.py`、`astar_jps.py` 等。）

---

## 标准 A*

### 算法简述
```
* 初始化open_set和close_set；
* 将起点加入open_set中，并设置优先级为0（优先级最高）；
* 如果open_set不为空，则从open_set中选取优先级最高的节点n：
    * 如果节点n为终点，则：
        * 从终点开始逐步追踪parent节点，一直达到起点；
        * 返回找到的结果路径，算法结束；
    * 如果节点n不是终点，则：
        * 将节点n从open_set中删除，并加入close_set中；
        * 遍历节点n所有的邻近节点：
            * 如果邻近节点m在close_set中，则：
                * 跳过，选取下一个邻近节点
            * 如果邻近节点m也不在open_set中，则：
                * 设置节点m的parent为节点n
                * 计算节点m的优先级
                * 将节点m加入open_set中
```
### 类与接口

- **`Node(pos, g, f)`**  
  搜索节点：`pos=(row, col)`，`g` 为已走代价，`f = g + h` 用于堆排序。

- **`AstarPlanner`**
  - `plan(map_matrix, start, goal)`  
    输入：栅格地图（0 可通行，1 障碍）、起点/终点 `(row, col)`。  
    输出：路径 `[(row, col), ...]` 或 `None`。

### 使用示例

```python
from astar import AstarPlanner

planner = AstarPlanner()
path = planner.plan(map_matrix, start=(r0, c0), goal=(r1, c1))
# path 为 None 表示无解
```

---

## A* 变体

以下小节用于记录本目录中 A* 各类变体的说明与注释（可自行增删）。

### 1. （变体名称，如加权 A*、JPS 等）

- **文件**：`xxx.py`
- **思路**：……
- **与标准 A* 的差异**：……
- **适用场景**：……

### 2. （可继续添加）

- **文件**：……
- **思路**：……
- **与标准 A* 的差异**：……
- **适用场景**：……

---

## 约定与说明

- **坐标系**：栅格为 `map[row, col]`，即先行后列；路径点均为 `(row, col)`。
- **地图约定**：`0` 表示可通行，`1` 表示障碍；起点/终点必须在可通行格且在地图范围内。
- **依赖**：`numpy`、标准库 `heapq`、`typing`。

---

## 更新记录

| 日期 | 内容 |
|------|------|
| （填写） | 初始 README，补充标准 A* 说明 |
| （填写） | 新增/修改某变体说明 |
