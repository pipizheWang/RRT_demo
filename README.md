# RRT & RRT* 二维路径规划算法

这是一个完整的Python实现，包含了**RRT (Rapidly-exploring Random Tree)** 和 **RRT*** 两种路径规划算法，用于二维空间中的路径规划问题。

## 🚀 功能特点

- **双算法支持**: RRT（快速探索）+ RRT*（渐近最优）
- **简单易用**: 核心算法逻辑清晰，容易理解和修改
- **可视化友好**: 内置matplotlib可视化功能，支持中文显示
- **障碍物支持**: 支持圆形障碍物的精确碰撞检测
- **参数可调**: 灵活的参数配置，适应不同应用场景
- **多种示例**: 从基础到高级的完整使用示例

## 📁 文件说明

| 文件 | 说明 | 算法 |
|------|------|------|
| `rrt.py` | RRT算法核心实现 | RRT |
| `rrt_star.py` | RRT*算法核心实现 | RRT* |
| `run_rrt.py` | RRT算法专用运行脚本 | RRT |
| `run_rrt_star.py` | RRT*算法专用运行脚本 | RRT* |
| `compare_rrt_rrt_star.py` | 两种算法对比程序 | RRT + RRT* |
| `RRT_vs_RRT_Star.md` | 详细算法对比文档 | 理论分析 |

## 📋 详细代码说明

### 🔷 核心算法实现

#### `rrt.py` - RRT算法核心
```python
# 主要类和方法
class Node:           # 树节点类
class RRT:            # RRT算法主类
  - random_sample()   # 随机采样
  - get_nearest_node() # 最近邻搜索
  - steer()          # 节点扩展
  - collision_check() # 碰撞检测
  - plan()           # 主要规划函数
  - visualize()      # 可视化功能
```

**特点**：
- ✅ 快速路径规划
- ✅ 简单直观的实现
- ✅ 适合实时应用
- ✅ 中文可视化支持

#### `rrt_star.py` - RRT*算法核心
```python
# 主要类和方法 (继承RRT的所有功能)
class RRTStar(RRT):   # RRT*算法主类
  - get_near_nodes()  # 🆕 邻近节点搜索
  - choose_parent()   # 🆕 最优父节点选择
  - rewire()          # 🆕 重新布线优化
  - update_children_cost() # 🆕 成本递归更新
  - plan()            # 🆕 包含优化的规划函数
```

**特点**：
- ✅ 渐近最优路径
- ✅ 动态路径优化
- ✅ 成本可视化
- ✅ 高质量路径保证

### 🚀 专用运行脚本

#### `run_rrt.py` - RRT专用脚本
```python
# 主要功能函数
def simple_rrt():              # 简单RRT示例
def custom_rrt():              # 自定义参数RRT
def compare_different_step_size(): # 步长对比测试
def maze_rrt():                # 迷宫环境挑战
def performance_test():        # 性能基准测试
def calculate_path_cost():     # 路径成本计算
```

**运行模式**：
1. **简单示例** - 默认参数快速体验
2. **自定义参数** - 交互式参数设置
3. **步长对比** - 可视化不同步长效果
4. **迷宫挑战** - 复杂环境测试
5. **性能测试** - 多配置性能基准

#### `run_rrt_star.py` - RRT*专用脚本
```python
# 主要功能函数
def simple_rrt_star():         # 简单RRT*示例
def custom_rrt_star():         # 自定义参数RRT*
def compare_different_radius(): # 搜索半径对比测试
```

**运行模式**：
1. **简单示例** - 默认参数展示RRT*优势
2. **自定义参数** - 交互式参数调优
3. **搜索半径对比** - 可视化不同半径效果

### 🔄 对比和分析工具

#### `compare_rrt_rrt_star.py` - 算法对比
```python
# 主要功能
def compare_algorithms():      # 性能和质量对比
def plot_result():            # 结果可视化
def explain_differences():     # 算法差异分析
def algorithm_steps_comparison(): # 步骤对比说明
```

**对比内容**：
- 📊 路径成本对比
- ⏱️ 运行时间分析
- 🌳 树结构可视化
- 📈 优化效果展示

#### `RRT_vs_RRT_Star.md` - 理论文档
**包含内容**：
- 🎯 核心区别总结
- 🔧 关键改动详解
- 📊 算法流程对比
- 🧮 复杂度分析
- 🎯 适用场景指导
- 🛠️ 参数调优建议

### 📦 辅助文件

#### `requirements.txt` - 依赖管理
```
numpy>=1.20.0
matplotlib>=3.3.0
```

#### `.gitignore` - 版本控制
- Python缓存文件忽略
- 临时文件过滤
- IDE配置排除

## 📦 依赖项

```bash
pip install numpy matplotlib
```

## 🌟 算法对比

| 特性 | RRT | RRT* |
|------|-----|------|
| **算法目标** | 快速找到可行路径 | 找到渐近最优路径 |
| **路径质量** | 通常较差，可能曲折 | 逐渐优化，接近最优 |
| **计算复杂度** | O(n) | O(n log n) |
| **运行速度** | 快速 | 较慢（优化开销） |
| **适用场景** | 实时应用、快速原型 | 高质量路径、离线规划 |

## 🎯 快速开始

### 1. 运行RRT算法

```bash
# 运行RRT专用脚本
python run_rrt.py
```

### 2. 运行RRT*算法

```bash
# 运行RRT*专用脚本
python run_rrt_star.py
```

### 3. 对比两种算法

```bash
# 运行算法对比程序
python compare_rrt_rrt_star.py
```

## 💻 基本使用

### RRT 算法使用

```python
from rrt import RRT

# 创建RRT实例
rrt = RRT(
    start=(1, 1),                    # 起点
    goal=(9, 9),                     # 终点
    obstacles=[(5, 5, 1)],           # 障碍物列表 (x, y, radius)
    map_size=(10, 10),               # 地图大小
    step_size=0.5,                   # 扩展步长
    max_iter=2000                    # 最大迭代次数
)

# 执行路径规划
path, nodes = rrt.plan()

# 可视化结果
rrt.visualize(path)
```

### RRT* 算法使用

```python
from rrt_star import RRTStar

# 创建RRT*实例
rrt_star = RRTStar(
    start=(1, 1),                    # 起点
    goal=(9, 9),                     # 终点
    obstacles=[(5, 5, 1)],           # 障碍物列表
    map_size=(10, 10),               # 地图大小
    step_size=0.4,                   # 扩展步长
    search_radius=1.2,               # 🆕 搜索半径
    max_iter=2000                    # 最大迭代次数
)

# 执行路径规划
path, nodes, cost = rrt_star.plan()  # 🆕 返回路径成本

# 可视化结果
rrt_star.visualize(path, cost)
```

## ⚙️ 参数说明

### 共同参数

- `start`: 起点坐标 (x, y)
- `goal`: 目标点坐标 (x, y)
- `obstacles`: 障碍物列表，格式：[(x, y, radius), ...]
- `map_size`: 地图大小 (width, height)，默认 (10, 10)
- `step_size`: 扩展步长，默认 0.5
- `max_iter`: 最大迭代次数，默认 1000

### RRT* 特有参数

- `search_radius`: 邻近节点搜索半径，影响优化效果
  - 较小值 (0.8-1.0): 局部优化，计算快
  - 较大值 (1.5-2.5): 全局优化，计算慢

## 🔧 算法流程对比

### RRT 算法流程
```
1. 随机采样 → 2. 最近邻搜索 → 3. 扩展节点 → 4. 碰撞检测 → 5. 添加节点
```

### RRT* 算法流程
```
1. 随机采样 → 2. 最近邻搜索 → 3. 扩展节点 → 4. 邻近节点搜索
                                                    ↓
5. 选择最优父节点 → 6. 添加节点 → 7. 重新布线邻近节点
```

**RRT* 的关键改进**：
- 🎯 **邻近节点搜索**: 在搜索半径内寻找多个候选父节点
- 🏆 **最优父节点选择**: 选择成本最低的路径连接
- 🔄 **重新布线**: 动态优化已有节点的连接关系
- 📈 **成本传播**: 递归更新子树的路径成本

## 🌈 中文字体配置

本实现已经自动配置中文字体支持：

- **自动检测系统**: 支持 macOS、Windows、Linux
- **智能字体选择**: 根据系统自动选择最佳中文字体
- **完美显示**: 图表标题、坐标轴标签、图例等完全支持中文

### 手动字体配置（如有需要）

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

## 📊 使用场景选择

### 选择 RRT 当：
- ✅ 需要快速找到可行解
- ✅ 实时应用（游戏、机器人避障）
- ✅ 计算资源有限
- ✅ 路径质量要求不高
- ✅ 原型开发和测试

### 选择 RRT* 当：
- ✅ 需要高质量路径
- ✅ 离线路径规划
- ✅ 路径成本很重要（如燃油消耗）
- ✅ 有充足的计算时间
- ✅ 商业应用和产品部署

## 🎮 运行示例

### 简单开始
```bash
# 最简单的方式：运行RRT*
python run_rrt_star.py
# 选择 "1" 运行简单示例
```

### 参数调优
```bash
# 运行RRT*自定义参数
python run_rrt_star.py
# 选择 "2" 输入自定义参数
```

### 性能对比
```bash
# 运行算法对比
python compare_rrt_rrt_star.py
```

## 🔬 示例结果说明

运行程序后的可视化图表：

- **蓝色细线**: 算法探索树的生长过程
- **绿色粗线**: 找到的最终路径
- **绿色圆点**: 起点位置
- **红色圆点**: 目标点位置
- **红色圆形**: 圆形障碍物
- **颜色深度**: 在RRT*中表示节点的路径成本

## 🛠️ 自定义和扩展

项目设计便于扩展，可以轻松修改：

1. **障碍物类型**: 当前支持圆形，可扩展为矩形、多边形等
2. **采样策略**: 可使用更智能的采样方法（如Informed RRT*）
3. **距离度量**: 可修改为曼哈顿距离、切比雪夫距离等
4. **维度扩展**: 算法结构支持扩展到三维空间
5. **优化算法**: 可添加路径平滑化、快捷方式等后处理

## 📈 性能优化建议

### RRT 参数调优
- `step_size`: 0.3-0.8 (复杂环境用小值)
- `max_iter`: 1000-3000 (根据地图复杂度)

### RRT* 参数调优
- `step_size`: 0.2-0.5 (比RRT稍小)
- `search_radius`: step_size * 2-3
- `max_iter`: 1500-5000 (需要更多迭代优化)

## 🤝 贡献和反馈

欢迎提出：
- Bug 报告
- 功能建议
- 性能优化
- 代码改进
- 文档完善

## 📚 进一步学习

- 📖 详细算法对比：查看 `RRT_vs_RRT_Star.md`
- 🎯 实际应用：机器人导航、游戏AI、无人机路径规划
- 🚀 高级变种：Informed RRT*、Anytime RRT*、RRT-Connect

---

**快速测试**: 
- RRT算法：`python run_rrt.py` 
- RRT*算法：`python run_rrt_star.py` 
- 算法对比：`python compare_rrt_rrt_star.py`

开始你的路径规划之旅！🎉