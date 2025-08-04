import numpy as np
import matplotlib.pyplot as plt
import random
import math
import platform

def setup_chinese_font():
    """设置matplotlib中文字体"""
    system = platform.system()
    
    # 设置中文字体
    if system == "Darwin":  # macOS
        fonts = ["Arial Unicode MS", "PingFang SC", "Helvetica"]
    elif system == "Windows":  # Windows
        fonts = ["SimHei", "Microsoft YaHei", "SimSun"]
    else:  # Linux
        fonts = ["DejaVu Sans", "WenQuanYi Micro Hei", "Droid Sans Fallback"]
    
    # 尝试设置字体
    for font in fonts:
        try:
            plt.rcParams['font.sans-serif'] = [font]
            plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
            break
        except:
            continue
    
    # 如果都不行，尝试系统默认中文字体
    try:
        if system == "Darwin":
            plt.rcParams['font.sans-serif'] = ['Arial Unicode MS']
        elif system == "Windows":
            plt.rcParams['font.sans-serif'] = ['Microsoft YaHei']
        else:
            plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False
    except:
        print("警告: 无法设置中文字体，中文可能显示为方框")

# 初始化中文字体设置
setup_chinese_font()

class Node:
    """RRT*树中的节点"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0  # 🆕 新增：从起点到该节点的累积成本

class RRTStar:
    """RRT*算法实现 - 渐近最优版本"""
    
    def __init__(self, start, goal, obstacles=None, map_size=(10, 10), step_size=0.5, 
                 search_radius=1.0, max_iter=1000):
        """
        初始化RRT*算法
        
        参数:
        start: 起点坐标 (x, y)
        goal: 目标点坐标 (x, y)
        obstacles: 障碍物列表，每个障碍物是一个圆形 [(x, y, radius), ...]
        map_size: 地图大小 (width, height)
        step_size: 扩展步长
        search_radius: 搜索半径（用于寻找邻近节点）🆕
        max_iter: 最大迭代次数
        """
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacles = obstacles if obstacles else []
        self.map_size = map_size
        self.step_size = step_size
        self.search_radius = search_radius  # 🆕 RRT*特有参数
        self.max_iter = max_iter
        self.node_list = [self.start]
        
        # 起点的成本为0
        self.start.cost = 0.0
        
    def random_sample(self):
        """随机采样一个点"""
        # 10%的概率采样目标点，90%的概率随机采样
        if random.random() < 0.1:
            return Node(self.goal.x, self.goal.y)
        else:
            x = random.uniform(0, self.map_size[0])
            y = random.uniform(0, self.map_size[1])
            return Node(x, y)
    
    def get_nearest_node(self, sample_node):
        """找到树中距离采样点最近的节点"""
        distances = [(node.x - sample_node.x)**2 + (node.y - sample_node.y)**2 
                    for node in self.node_list]
        min_index = distances.index(min(distances))
        return self.node_list[min_index]
    
    def steer(self, from_node, to_node):
        """从from_node向to_node方向扩展step_size距离"""
        distance = math.sqrt((to_node.x - from_node.x)**2 + (to_node.y - from_node.y)**2)
        
        if distance <= self.step_size:
            new_node = Node(to_node.x, to_node.y)
        else:
            # 计算单位方向向量
            theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
            new_x = from_node.x + self.step_size * math.cos(theta)
            new_y = from_node.y + self.step_size * math.sin(theta)
            new_node = Node(new_x, new_y)
        
        return new_node
    
    def collision_check(self, from_node, to_node):
        """检查从from_node到to_node的路径是否与障碍物碰撞"""
        # 检查终点是否在地图范围内
        if (to_node.x < 0 or to_node.x > self.map_size[0] or 
            to_node.y < 0 or to_node.y > self.map_size[1]):
            return False
        
        # 检查路径是否与圆形障碍物碰撞
        for obstacle in self.obstacles:
            obs_x, obs_y, obs_radius = obstacle
            
            # 简单的点到圆的距离检查
            dist_to_obstacle = math.sqrt((to_node.x - obs_x)**2 + (to_node.y - obs_y)**2)
            if dist_to_obstacle <= obs_radius:
                return False
                
            # 检查线段与圆的碰撞（更精确的检查）
            if self.line_circle_collision(from_node, to_node, obstacle):
                return False
        
        return True
    
    def line_circle_collision(self, from_node, to_node, obstacle):
        """检查线段是否与圆形障碍物碰撞"""
        obs_x, obs_y, obs_radius = obstacle
        
        # 线段的方向向量
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        
        # 从线段起点到圆心的向量
        fx = from_node.x - obs_x
        fy = from_node.y - obs_y
        
        # 二次方程系数
        a = dx*dx + dy*dy
        if a == 0:
            return False
            
        b = 2*(fx*dx + fy*dy)
        c = (fx*fx + fy*fy) - obs_radius*obs_radius
        
        discriminant = b*b - 4*a*c
        
        if discriminant < 0:
            return False  # 无交点
        
        # 计算交点参数
        discriminant = math.sqrt(discriminant)
        t1 = (-b - discriminant) / (2*a)
        t2 = (-b + discriminant) / (2*a)
        
        # 检查交点是否在线段上
        return (0 <= t1 <= 1) or (0 <= t2 <= 1)
    
    def distance(self, node1, node2):
        """🆕 计算两节点间的欧几里得距离"""
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
    
    def get_near_nodes(self, new_node):
        """🆕 RRT*核心：获取新节点搜索半径内的所有邻近节点"""
        near_nodes = []
        for node in self.node_list:
            if self.distance(node, new_node) <= self.search_radius:
                near_nodes.append(node)
        return near_nodes
    
    def choose_parent(self, new_node, near_nodes):
        """🆕 RRT*核心：为新节点选择成本最低的父节点"""
        if not near_nodes:
            return None
        
        min_cost = float('inf')
        best_parent = None
        
        for near_node in near_nodes:
            # 检查连接是否无碰撞
            if self.collision_check(near_node, new_node):
                # 计算通过这个邻近节点到新节点的总成本
                cost = near_node.cost + self.distance(near_node, new_node)
                if cost < min_cost:
                    min_cost = cost
                    best_parent = near_node
        
        if best_parent is not None:
            new_node.parent = best_parent
            new_node.cost = min_cost
            return best_parent
        
        return None
    
    def rewire(self, new_node, near_nodes):
        """🆕 RRT*核心：重新布线 - 检查是否可以通过新节点改善其他节点的路径"""
        for near_node in near_nodes:
            # 跳过新节点的父节点
            if near_node == new_node.parent:
                continue
            
            # 检查连接是否无碰撞
            if self.collision_check(new_node, near_node):
                # 计算通过新节点到达这个邻近节点的成本
                new_cost = new_node.cost + self.distance(new_node, near_node)
                
                # 如果新路径更优，则重新布线
                if new_cost < near_node.cost:
                    near_node.parent = new_node
                    near_node.cost = new_cost
                    # 递归更新子树的成本
                    self.update_children_cost(near_node)
    
    def update_children_cost(self, parent_node):
        """🆕 递归更新子树中所有节点的成本"""
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = parent_node.cost + self.distance(parent_node, node)
                self.update_children_cost(node)
    
    def is_near_goal(self, node, threshold=0.5):
        """检查节点是否接近目标点"""
        distance = math.sqrt((node.x - self.goal.x)**2 + (node.y - self.goal.y)**2)
        return distance <= threshold
    
    def get_path(self, goal_node):
        """从目标节点回溯得到完整路径"""
        path = []
        current_node = goal_node
        
        while current_node is not None:
            path.append([current_node.x, current_node.y])
            current_node = current_node.parent
        
        return path[::-1]  # 反转路径，从起点到终点
    
    def get_path_cost(self, goal_node):
        """🆕 获取路径的总成本"""
        return goal_node.cost
    
    def plan(self):
        """🆕 执行RRT*路径规划"""
        print("开始RRT*路径规划...")
        
        for i in range(self.max_iter):
            # 1. 随机采样
            sample_node = self.random_sample()
            
            # 2. 找到最近节点
            nearest_node = self.get_nearest_node(sample_node)
            
            # 3. 扩展新节点
            new_node = self.steer(nearest_node, sample_node)
            
            # 4. 🆕 获取邻近节点
            near_nodes = self.get_near_nodes(new_node)
            
            # 5. 🆕 选择最优父节点
            best_parent = self.choose_parent(new_node, near_nodes)
            
            if best_parent is not None:
                # 6. 添加新节点到树中
                self.node_list.append(new_node)
                
                # 7. 🆕 重新布线
                self.rewire(new_node, near_nodes)
                
                # 8. 检查是否到达目标
                if self.is_near_goal(new_node):
                    # 尝试直接连接到目标点
                    final_node = self.steer(new_node, self.goal)
                    if self.collision_check(new_node, final_node):
                        final_node.parent = new_node
                        final_node.cost = new_node.cost + self.distance(new_node, final_node)
                        self.node_list.append(final_node)
                        path = self.get_path(final_node)
                        path_cost = self.get_path_cost(final_node)
                        print(f"找到路径！迭代次数: {i+1}, 路径成本: {path_cost:.2f}")
                        return path, self.node_list, path_cost
        
        print("未找到路径")
        return None, self.node_list, float('inf')
    
    def visualize(self, path=None, path_cost=None):
        """🆕 可视化RRT*树和路径，显示成本信息"""
        plt.figure(figsize=(12, 8))
        
        # 绘制障碍物
        for obstacle in self.obstacles:
            obs_x, obs_y, obs_radius = obstacle
            circle = plt.Circle((obs_x, obs_y), obs_radius, color='red', alpha=0.7)
            plt.gca().add_patch(circle)
        
        # 绘制RRT*树
        for node in self.node_list:
            if node.parent:
                # 根据成本设置颜色深度
                alpha = min(0.7, 0.1 + node.cost / 20)
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 
                        'b-', alpha=alpha, linewidth=0.5)
        
        # 绘制所有节点，颜色表示成本
        costs = [node.cost for node in self.node_list]
        max_cost = max(costs) if costs else 1
        
        for node in self.node_list:
            # 成本越高，颜色越深
            color_intensity = node.cost / max_cost if max_cost > 0 else 0
            plt.scatter(node.x, node.y, c=color_intensity, cmap='Blues', 
                       s=20, alpha=0.6, vmin=0, vmax=1)
        
        # 绘制起点和终点
        plt.plot(self.start.x, self.start.y, 'go', markersize=10, label='起点')
        plt.plot(self.goal.x, self.goal.y, 'ro', markersize=10, label='目标点')
        
        # 绘制路径
        if path:
            path_x = [point[0] for point in path]
            path_y = [point[1] for point in path]
            plt.plot(path_x, path_y, 'g-', linewidth=4, label=f'RRT*路径 (成本:{path_cost:.2f})', alpha=0.8)
        
        plt.xlim(0, self.map_size[0])
        plt.ylim(0, self.map_size[1])
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.title('RRT* 路径规划算法 (渐近最优)', fontsize=14)
        plt.xlabel('X坐标')
        plt.ylabel('Y坐标')
        
        # 添加算法参数信息
        info_text = f"""RRT* 参数:
• 搜索半径: {self.search_radius}
• 步长: {self.step_size}
• 节点数: {len(self.node_list)}
• 地图大小: {self.map_size[0]}×{self.map_size[1]}"""
        
        plt.text(0.02, 0.98, info_text, transform=plt.gca().transAxes, 
                fontsize=10, verticalalignment='top',
                bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgreen", alpha=0.8))
        
        plt.colorbar(plt.cm.ScalarMappable(cmap='Blues'), label='节点成本')
        plt.tight_layout()
        plt.show()

def main():
    """主函数示例"""
    # 设置起点和终点
    start = (1, 1)
    goal = (9, 9)
    
    # 定义圆形障碍物 (x, y, radius)
    obstacles = [
        (3, 3, 1),
        (6, 4, 0.8),
        (4, 7, 1.2),
        (7, 6, 0.9)
    ]
    
    # 创建RRT*实例
    rrt_star = RRTStar(start=start, 
                       goal=goal, 
                       obstacles=obstacles, 
                       map_size=(10, 10), 
                       step_size=0.5, 
                       search_radius=1.5,  # 🆕 搜索半径参数
                       max_iter=2000)
    
    # 执行路径规划
    path, nodes, path_cost = rrt_star.plan()
    
    # 可视化结果
    rrt_star.visualize(path, path_cost)
    
    if path:
        print("RRT*路径找到！")
        print(f"路径成本: {path_cost:.2f}")
        print(f"路径长度: {len(path)} 个点")
        print(f"总节点数: {len(nodes)}")

if __name__ == "__main__":
    main()