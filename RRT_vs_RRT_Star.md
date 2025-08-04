# RRT vs RRT* 算法详细对比

## 🎯 核心区别

| 特性 | RRT | RRT* |
|------|-----|------|
| **算法目标** | 快速找到可行路径 | 找到渐近最优路径 |
| **路径质量** | 通常较差 | 逐渐优化，接近最优 |
| **时间复杂度** | O(n) | O(n log n) |
| **空间复杂度** | O(n) | O(n) |
| **收敛性** | 概率完备 | 概率完备 + 渐近最优 |

## 🔧 RRT* 的关键改动

### 1. 节点结构扩展

```python
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0  # 🆕 新增：累积成本
```

**改动说明**：
- 每个节点增加了 `cost` 字段
- 记录从起点到该节点的累积路径成本
- 用于后续的路径优化决策

### 2. 搜索半径参数

```python
def __init__(self, ..., search_radius=1.0):
    self.search_radius = search_radius  # 🆕 RRT*特有参数
```

**改动说明**：
- 定义邻近节点的搜索范围
- 影响算法的优化效果和计算开销
- 通常设置为步长的2-3倍

### 3. 邻近节点搜索

```python
def get_near_nodes(self, new_node):
    """🆕 获取搜索半径内的所有邻近节点"""
    near_nodes = []
    for node in self.node_list:
        if self.distance(node, new_node) <= self.search_radius:
            near_nodes.append(node)
    return near_nodes
```

**与RRT的区别**：
- RRT：只考虑1个最近邻节点
- RRT*：考虑搜索半径内的所有节点

### 4. 最优父节点选择

```python
def choose_parent(self, new_node, near_nodes):
    """🆕 为新节点选择成本最低的父节点"""
    min_cost = float('inf')
    best_parent = None
    
    for near_node in near_nodes:
        if self.collision_check(near_node, new_node):
            cost = near_node.cost + self.distance(near_node, new_node)
            if cost < min_cost:
                min_cost = cost
                best_parent = near_node
    
    return best_parent
```

**与RRT的区别**：
- RRT：直接使用最近邻节点作为父节点
- RRT*：在所有可行的邻近节点中选择成本最低的

### 5. 重新布线操作

```python
def rewire(self, new_node, near_nodes):
    """🆕 重新布线 - 优化邻近节点的路径"""
    for near_node in near_nodes:
        if self.collision_check(new_node, near_node):
            new_cost = new_node.cost + self.distance(new_node, near_node)
            if new_cost < near_node.cost:
                near_node.parent = new_node  # 重新连接
                near_node.cost = new_cost
                self.update_children_cost(near_node)  # 递归更新
```

**RRT完全没有这个步骤**：
- 检查是否可以通过新节点改善其他节点的路径
- 如果可以，就重新连接父子关系
- 这是RRT*实现渐近最优的关键机制

### 6. 成本递归更新

```python
def update_children_cost(self, parent_node):
    """🆕 递归更新子树中所有节点的成本"""
    for node in self.node_list:
        if node.parent == parent_node:
            node.cost = parent_node.cost + self.distance(parent_node, node)
            self.update_children_cost(node)  # 递归调用
```

**RRT不需要这个操作**：
- 当重新布线改变了某个节点的父节点时
- 需要递归更新其所有子孙节点的成本
- 确保成本信息的一致性

## 📊 算法流程对比

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

## 🧮 复杂度分析

### 时间复杂度

**RRT**：
- 最近邻搜索：O(n)
- 总复杂度：O(n²)

**RRT***：
- 邻近节点搜索：O(n)
- 重新布线：O(k) where k = 邻近节点数
- 总复杂度：O(n² log n)

### 空间复杂度

**两者都是 O(n)**：
- 存储所有生成的节点
- RRT*需要额外存储成本信息

## 🎯 适用场景

### 选择 RRT 当：
- ✅ 需要快速找到可行解
- ✅ 实时应用（如游戏、机器人避障）
- ✅ 计算资源有限
- ✅ 路径质量要求不高

### 选择 RRT* 当：
- ✅ 需要高质量路径
- ✅ 离线路径规划
- ✅ 路径成本很重要（如燃油消耗）
- ✅ 有充足的计算时间

## 📈 性能对比

### 路径质量
- **RRT**：路径可能很曲折，成本较高
- **RRT***：随着迭代次数增加，路径逐渐优化

### 计算时间
- **RRT**：通常更快找到第一条路径
- **RRT***：需要更多时间，但路径更优

### 收敛特性
- **RRT**：找到路径后不再优化
- **RRT***：持续优化直到达到最大迭代次数

## 🛠️ 参数调优

### search_radius 的选择
```python
# 经验公式
search_radius = min(
    step_size * 2,  # 不超过步长的2倍
    γ * (log(n)/n)^(1/d)  # 渐近最优半径公式，d为维度
)
```

### 推荐设置
```python
# 快速规划
search_radius = step_size * 1.5

# 高质量路径
search_radius = step_size * 2.5

# 复杂环境
search_radius = step_size * 3.0
```

## 🔬 理论保证

### RRT
- **概率完备性**：如果解存在，算法最终会找到
- **无最优性保证**：找到的路径可能远非最优

### RRT*
- **概率完备性**：继承自RRT
- **渐近最优性**：随着采样点增加，路径成本趋向最优解

## 🚀 进一步优化

### RRT* 的变种
1. **Informed RRT***：使用启发式信息加速搜索
2. **Anytime RRT***：可以随时中断并返回当前最佳路径
3. **RRT*-Smart**：智能采样策略
4. **Batch Informed RRT***：批量处理优化

### 实现优化
1. **KD树**：加速最近邻搜索
2. **延迟重新布线**：减少不必要的重新布线
3. **自适应搜索半径**：根据节点密度调整半径