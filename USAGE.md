# SCP轨迹规划使用指南

基于RRT/RRT*路径的SCP轨迹优化系统

## 快速开始

### 1. 安装依赖
```bash
pip install -r requirements.txt
```

### 2. 运行集成系统
```bash
python run_scp_trajectory.py
```

选择运行模式：
- `1` - RRT + SCP轨迹优化
- `2` - RRT* + SCP轨迹优化  
- `3` - 完整算法对比
- `4` - 自定义演示

## 编程接口

### 基本用法
```python
from rrt import RRT
from scp_trajectory import SCPTrajectoryPlanner

# 1. 路径规划
rrt = RRT(start=(1,1), goal=(9,9), obstacles=[(3,3,1), (6,6,0.8)])
path, nodes = rrt.plan()

# 2. 轨迹优化
scp = SCPTrajectoryPlanner(
    max_velocity=2.0,
    max_acceleration=1.5,
    obstacles=[(3,3,1), (6,6,0.8)],
    time_horizon=8.0,
    num_segments=30
)

scp.set_path_waypoints(path)
trajectory, success = scp.plan_trajectory()

# 3. 可视化
if success:
    scp.visualize_trajectory()
```

### 参数说明

#### SCPTrajectoryPlanner参数
- `max_velocity`: 最大速度 (m/s)
- `max_acceleration`: 最大加速度 (m/s²)
- `obstacles`: 障碍物 [(x, y, radius), ...]
- `time_horizon`: 轨迹时间跨度 (s)
- `num_segments`: 时间离散化段数

#### plan_trajectory参数
- `max_iterations`: 最大迭代次数 (默认10)
- `use_staged_optimization`: 是否使用分阶段优化 (默认True)

## 输出结果

### 轨迹格式
```python
# trajectory.shape = (num_segments+1, 4)
# 每行: [x, y, vx, vy]
trajectory[i] = [位置x, 位置y, 速度x, 速度y]
```

### 获取特定时间点状态
```python
state = scp.get_trajectory_at_time(t)  # [x, y, vx, vy]
```

## 算法特点

- **路径跟踪**: 强化waypoint跟踪确保贴近原始路径
- **动力学约束**: 满足速度、加速度、曲率等物理限制
- **平滑轨迹**: 生成连续可微的运动轨迹
- **自适应参数**: 根据路径长度自动调整时间参数
- **分阶段优化**: 先放宽约束后收紧，提高收敛性

## 应用场景

- 🤖 移动机器人路径执行
- 🚗 自动驾驶轨迹规划
- ✈️ 无人机航迹优化
- 🦾 机械臂运动规划