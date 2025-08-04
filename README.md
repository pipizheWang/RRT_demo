# RRT (Rapidly-exploring Random Tree) 二维路径规划算法

这是一个简单的Python实现，用于二维空间中的RRT路径规划算法。

## 功能特点

- **简单易用**: 核心算法逻辑清晰，容易理解和修改
- **可视化**: 内置matplotlib可视化功能，可以看到树的生长过程和最终路径
- **障碍物支持**: 支持圆形障碍物的碰撞检测
- **参数可调**: 可以调整步长、迭代次数等参数
- **多种示例**: 提供从简单到复杂的多个使用示例

## 文件说明

- `rrt.py`: RRT算法的核心实现
- `example.py`: 使用示例和演示程序
- `README.md`: 说明文档

## 依赖项

```bash
pip install numpy matplotlib
```

## 中文字体配置

本实现已经自动配置了中文字体支持，解决了matplotlib中文显示问题：

- **自动检测系统**：支持 macOS、Windows、Linux
- **智能字体选择**：根据系统自动选择最佳中文字体
- **兼容性良好**：在不同操作系统下都能正常显示中文


### 如果中文显示异常

如果仍然显示方框，请尝试：

**macOS**:
```python
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False
```

**Windows**:
```python
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False
```

**Linux**:
```bash
# 安装中文字体
sudo apt-get install fonts-wqy-microhei
# 更新字体缓存
fc-cache -fv
```

## 快速开始

### 1. 运行示例程序

```bash
python example.py
```

程序会提供多个示例选择：
1. 简单示例（无障碍物）
2. 障碍物示例  
3. 迷宫示例
4. 参数对比示例
5. 运行所有示例

### 2. 基本使用

```python
from rrt import RRT

# 定义起点和终点
start = (1, 1)
goal = (9, 9)

# 定义圆形障碍物 (x, y, radius)
obstacles = [
    (3, 3, 1),
    (6, 4, 0.8),
    (4, 7, 1.2),
]

# 创建RRT实例
rrt = RRT(start=start, 
          goal=goal, 
          obstacles=obstacles, 
          map_size=(10, 10), 
          step_size=0.5, 
          max_iter=2000)

# 执行路径规划
path, nodes = rrt.plan()

# 可视化结果
rrt.visualize(path)
```

## 参数说明

### RRT类初始化参数

- `start`: 起点坐标 (x, y)
- `goal`: 目标点坐标 (x, y)  
- `obstacles`: 障碍物列表，每个障碍物为 (x, y, radius)
- `map_size`: 地图大小 (width, height)，默认 (10, 10)
- `step_size`: 扩展步长，默认 0.5
- `max_iter`: 最大迭代次数，默认 1000

### 算法流程

1. **随机采样**: 在地图中随机选择一个点（10%概率选择目标点）
2. **最近邻搜索**: 找到树中距离采样点最近的节点
3. **扩展**: 从最近节点向采样点方向扩展固定步长
4. **碰撞检测**: 检查新路径是否与障碍物碰撞
5. **添加节点**: 如果无碰撞，将新节点加入树中
6. **目标检测**: 检查是否到达目标点附近
7. **重复**: 继续迭代直到找到路径或达到最大迭代次数

## 算法特点

### 优点
- 概率完备性：如果解存在，算法最终会找到
- 适用于高维空间
- 实现简单，计算效率高
- 对复杂环境有很好的探索能力

### 缺点
- 路径通常不是最优的
- 收敛速度可能较慢
- 生成的路径可能需要后处理（平滑化）

## 自定义和扩展

你可以轻松修改代码来适应特定需求：

1. **修改障碍物类型**: 当前支持圆形，可以扩展为矩形、多边形等
2. **改进采样策略**: 可以使用更智能的采样方法
3. **路径优化**: 添加路径平滑化或优化步骤
4. **三维扩展**: 算法结构支持扩展到三维空间

## 示例结果

运行程序后，你会看到：
- 蓝色细线：RRT探索树
- 绿色粗线：找到的路径
- 绿色圆点：起点
- 红色圆点：目标点
- 红色圆形：障碍物

## 问题和改进

如果你在使用过程中遇到问题或有改进建议，欢迎随时提出！