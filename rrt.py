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
    """RRT树中的节点"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class RRT:
    """二维空间RRT算法实现"""
    
    def __init__(self, start, goal, obstacles=None, map_size=(10, 10), step_size=0.5, max_iter=1000):
        """
        初始化RRT算法
        
        参数:
        start: 起点坐标 (x, y)
        goal: 目标点坐标 (x, y)
        obstacles: 障碍物列表，每个障碍物是一个圆形 [(x, y, radius), ...]
        map_size: 地图大小 (width, height)
        step_size: 扩展步长
        max_iter: 最大迭代次数
        """
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacles = obstacles if obstacles else []
        self.map_size = map_size
        self.step_size = step_size
        self.max_iter = max_iter
        self.node_list = [self.start]
        
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
            # 🔧 修复：即使距离很近，也要创建新节点并设置父节点关系
            new_node = Node(to_node.x, to_node.y)
            new_node.parent = from_node
            return new_node
        
        # 计算单位方向向量
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = from_node.x + self.step_size * math.cos(theta)
        new_y = from_node.y + self.step_size * math.sin(theta)
        
        new_node = Node(new_x, new_y)
        new_node.parent = from_node
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
    
    def plan(self):
        """执行RRT路径规划"""
        for i in range(self.max_iter):
            # 1. 随机采样
            sample_node = self.random_sample()
            
            # 2. 找到最近节点
            nearest_node = self.get_nearest_node(sample_node)
            
            # 3. 扩展新节点
            new_node = self.steer(nearest_node, sample_node)
            
            # 4. 碰撞检测
            if self.collision_check(nearest_node, new_node):
                self.node_list.append(new_node)
                
                # 5. 检查是否到达目标
                if self.is_near_goal(new_node):
                    # 尝试直接连接到目标点
                    final_node = self.steer(new_node, self.goal)
                    if self.collision_check(new_node, final_node):
                        # 🔧 确保final_node有正确的父节点关系（steer已经设置了）
                        self.node_list.append(final_node)
                        path = self.get_path(final_node)
                        print(f"找到路径！迭代次数: {i+1}")
                        return path, self.node_list
        
        print("未找到路径")
        return None, self.node_list
    
    def visualize(self, path=None):
        """可视化RRT树和路径"""
        plt.figure(figsize=(10, 8))
        
        # 绘制障碍物
        for obstacle in self.obstacles:
            obs_x, obs_y, obs_radius = obstacle
            circle = plt.Circle((obs_x, obs_y), obs_radius, color='red', alpha=0.7)
            plt.gca().add_patch(circle)
        
        # 绘制RRT树
        for node in self.node_list:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'b-', alpha=0.3, linewidth=0.5)
        
        # 绘制所有节点
        x_coords = [node.x for node in self.node_list]
        y_coords = [node.y for node in self.node_list]
        plt.plot(x_coords, y_coords, 'bo', markersize=2, alpha=0.6)
        
        # 绘制起点和终点
        plt.plot(self.start.x, self.start.y, 'go', markersize=8, label='起点')
        plt.plot(self.goal.x, self.goal.y, 'ro', markersize=8, label='目标点')
        
        # 绘制路径
        if path:
            path_x = [point[0] for point in path]
            path_y = [point[1] for point in path]
            plt.plot(path_x, path_y, 'g-', linewidth=3, label='规划路径')
        
        plt.xlim(0, self.map_size[0])
        plt.ylim(0, self.map_size[1])
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.title('RRT 路径规划算法')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.axis('equal')
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
    
    if path:
        print("路径找到！路径点:")
        for i, point in enumerate(path):
            print(f"  {i}: ({point[0]:.2f}, {point[1]:.2f})")
        print(f"路径长度: {len(path)} 个点")

if __name__ == "__main__":
    main()