# SCP轨迹规划模块

基于Sequential Convex Programming (SCP)的轨迹优化模块，用于将RRT/RRT*算法生成的离散路径点优化为满足动力学约束的平滑连续轨迹。

## 🎯 功能特点

- **路径平滑化**: 将离散的路径点转换为连续平滑的轨迹
- **动力学约束**: 满足速度、加速度、曲率等物理限制
- **障碍物避障**: 保持原有的避障特性
- **时间参数化**: 生成时间参数化的轨迹，便于实际执行
- **多算法集成**: 支持RRT和RRT*作为初始路径规划器

## 📋 新增文件

1. **`scp_trajectory.py`** - SCP轨迹规划核心模块
2. **`run_scp_trajectory.py`** - 集成运行脚本
3. **`SCP_README.md`** - 本说明文档

## 🚀 使用方法

### 1. 安装依赖

```bash
pip install -r requirements.txt
```

新增依赖：`scipy>=1.7.0` (用于数值优化)

### 2. 基本使用

#### 方式一：直接运行集成脚本

```bash
python run_scp_trajectory.py
```

选择运行模式：
- `1` - RRT + SCP轨迹优化
- `2` - RRT* + SCP轨迹优化  
- `3` - 完整算法对比 (RRT vs RRT* vs RRT+SCP vs RRT*+SCP)
- `4` - 自定义参数演示

#### 方式二：编程调用

```python
from rrt import RRT
from scp_trajectory import SCPTrajectoryPlanner

# 1. 使用RRT进行路径规划
rrt = RRT(start=(1,1), goal=(9,9), obstacles=[(3,3,1), (6,6,0.8)])
path, nodes = rrt.plan()

# 2. 使用SCP优化轨迹
scp_planner = SCPTrajectoryPlanner(
    max_velocity=2.0,      # 最大速度限制
    max_acceleration=1.0,  # 最大加速度限制
    max_curvature=0.5,     # 最大曲率限制
    obstacles=[(3,3,1), (6,6,0.8)],
    time_horizon=10.0,     # 轨迹总时间
    num_segments=50        # 时间离散化段数
)

scp_planner.set_path_waypoints(path)
trajectory, success = scp_planner.plan_trajectory()

# 3. 可视化结果
if success:
    scp_planner.visualize_trajectory()
    scp_planner.plot_trajectory_profiles()
```

## ⚙️ 主要参数说明

### SCPTrajectoryPlanner 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_velocity` | 2.0 | 最大速度约束 (m/s) |
| `max_acceleration` | 1.0 | 最大加速度约束 (m/s²) |
| `max_curvature` | 0.5 | 最大曲率约束 (1/m) |
| `obstacles` | None | 障碍物列表 [(x, y, radius), ...] |
| `time_horizon` | 10.0 | 轨迹总时间 (s) |
| `num_segments` | 50 | 时间离散化段数 |

### 优化参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_iterations` | 10 | SCP最大迭代次数 |
| `tolerance` | 1e-4 | 收敛容差 |

## 📊 算法对比

运行完整对比模式可以看到四种算法的性能差异：

| 算法 | 特点 | 适用场景 |
|------|------|----------|
| **RRT** | 快速、非最优 | 简单环境，快速规划 |
| **RRT*** | 渐近最优 | 复杂环境，路径质量要求高 |
| **RRT+SCP** | 平滑轨迹、满足动力学约束 | 机器人实际执行 |
| **RRT*+SCP** | 高质量平滑轨迹 | 对路径质量和平滑度都要求高的场景 |

## 🎨 可视化功能

### 1. 轨迹对比图
- 显示原始路径点 vs 优化后的连续轨迹
- 障碍物环境
- 速度向量可视化

### 2. 轨迹剖面图
- 位置随时间变化
- 速度随时间变化
- 加速度随时间变化
- 轨迹曲率随时间变化

### 3. 性能对比图
- 多算法路径长度对比
- 计算时间对比
- 节点数统计

## 💡 应用场景

1. **移动机器人路径规划**: 从抽象路径到可执行轨迹
2. **无人车导航**: 满足车辆动力学约束的平滑路径
3. **无人机航迹规划**: 考虑速度和加速度限制的飞行轨迹
4. **机械臂路径规划**: 关节空间的平滑运动轨迹

## 🔧 技术细节

### SCP算法流程

1. **初始化**: 基于RRT/RRT*路径生成初始轨迹猜测
2. **目标函数**: 最小化平滑度 + 能耗 + 时间 + waypoint跟踪误差
3. **约束条件**: 
   - 速度约束: |v| ≤ v_max
   - 加速度约束: |a| ≤ a_max  
   - 动力学一致性: 位置-速度积分关系
   - 障碍物避障: 安全距离约束
4. **迭代优化**: 使用SLSQP求解器进行序列二次规划
5. **收敛检查**: 轨迹变化小于阈值时停止

### 数学模型

轨迹状态: `x(t) = [px(t), py(t), vx(t), vy(t)]^T`

目标函数:
```
J = w1∑|a(t)|² + w2∑|v(t)|² + w3∑|x_waypoint - x(t)|²
```

约束条件:
```
|v(t)| ≤ v_max
|a(t)| ≤ a_max  
px(t+dt) = px(t) + vx(t)×dt
py(t+dt) = py(t) + vy(t)×dt
dist(x(t), obstacles) ≥ safety_margin
```

## 🚨 注意事项

1. **依赖安装**: 确保安装了scipy>=1.7.0
2. **参数调试**: 根据具体应用场景调整速度、加速度等约束参数
3. **收敛性**: 如果优化不收敛，可以尝试：
   - 增加迭代次数
   - 调整时间跨度
   - 放宽约束条件
   - 增加时间离散化段数

## 📈 性能建议

- **快速原型**: 使用RRT+SCP，计算速度快
- **高质量路径**: 使用RRT*+SCP，路径质量好
- **实时应用**: 减少`num_segments`和`max_iterations`
- **离线规划**: 可以使用更多的迭代次数和更细的时间离散化

## 🤝 扩展开发

模块设计具有良好的扩展性，可以方便地添加：

1. **新的约束类型** (如角速度约束)
2. **不同的目标函数** (如最小时间、最小能耗)
3. **其他优化算法** (如interior point方法)
4. **3D空间扩展** (添加z轴坐标)

---

*本模块与现有的RRT/RRT*代码完全兼容，保持了项目的整体结构不变。*