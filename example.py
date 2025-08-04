#!/usr/bin/env python3
"""
RRT算法使用示例
"""

from rrt import RRT, setup_chinese_font
import matplotlib.pyplot as plt

# 确保中文字体设置生效
setup_chinese_font()

def simple_example():
    """简单示例：无障碍物环境"""
    print("=== 简单示例：无障碍物环境 ===")
    
    start = (1, 1)
    goal = (8, 8)
    
    rrt = RRT(start=start, goal=goal, map_size=(10, 10))
    path, nodes = rrt.plan()
    rrt.visualize(path)

def obstacle_example():
    """复杂示例：有障碍物环境"""
    print("=== 复杂示例：有障碍物环境 ===")
    
    start = (1, 1)
    goal = (9, 9)
    
    # 定义几个圆形障碍物
    obstacles = [
        (3, 3, 1),      # 中心(3,3), 半径1
        (6, 4, 0.8),    # 中心(6,4), 半径0.8
        (4, 7, 1.2),    # 中心(4,7), 半径1.2
        (7, 6, 0.9)     # 中心(7,6), 半径0.9
    ]
    
    rrt = RRT(start=start, 
              goal=goal, 
              obstacles=obstacles, 
              map_size=(10, 10), 
              step_size=0.3,      # 较小的步长
              max_iter=3000)      # 更多迭代次数
    
    path, nodes = rrt.plan()
    rrt.visualize(path)

def maze_example():
    """迷宫示例：更复杂的环境"""
    print("=== 迷宫示例：复杂环境 ===")
    
    start = (0.5, 0.5)
    goal = (9.5, 9.5)
    
    # 创建一个简单的迷宫环境
    obstacles = [
        # 水平墙壁
        (2, 2, 0.3), (3, 2, 0.3), (4, 2, 0.3),
        (6, 4, 0.3), (7, 4, 0.3), (8, 4, 0.3),
        (1, 6, 0.3), (2, 6, 0.3), (3, 6, 0.3),
        (5, 8, 0.3), (6, 8, 0.3), (7, 8, 0.3),
        
        # 垂直墙壁
        (4, 3, 0.3), (4, 4, 0.3), (4, 5, 0.3),
        (6, 1, 0.3), (6, 2, 0.3), (6, 3, 0.3),
        (2, 7, 0.3), (2, 8, 0.3), (2, 9, 0.3),
        (8, 5, 0.3), (8, 6, 0.3), (8, 7, 0.3),
    ]
    
    rrt = RRT(start=start, 
              goal=goal, 
              obstacles=obstacles, 
              map_size=(10, 10), 
              step_size=0.2,      # 小步长适应复杂环境
              max_iter=5000)      # 更多迭代次数
    
    path, nodes = rrt.plan()
    rrt.visualize(path)

def custom_parameters():
    """自定义参数示例"""
    print("=== 自定义参数示例 ===")
    
    start = (1, 5)
    goal = (9, 5)
    
    obstacles = [
        (5, 5, 2),  # 大障碍物挡在中间
    ]
    
    # 不同参数设置的比较
    configs = [
        {"step_size": 0.2, "max_iter": 1000, "title": "小步长"},
        {"step_size": 1.0, "max_iter": 1000, "title": "大步长"},
    ]
    
    plt.figure(figsize=(15, 6))
    
    for i, config in enumerate(configs):
        plt.subplot(1, 2, i+1)
        
        rrt = RRT(start=start, 
                  goal=goal, 
                  obstacles=obstacles, 
                  map_size=(10, 10), 
                  step_size=config["step_size"], 
                  max_iter=config["max_iter"])
        
        path, nodes = rrt.plan()
        
        # 手动绘制（不使用rrt.visualize()）
        for obstacle in obstacles:
            obs_x, obs_y, obs_radius = obstacle
            circle = plt.Circle((obs_x, obs_y), obs_radius, color='red', alpha=0.7)
            plt.gca().add_patch(circle)
        
        # 绘制RRT树
        for node in nodes:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'b-', alpha=0.3, linewidth=0.5)
        
        plt.plot(start[0], start[1], 'go', markersize=8, label='起点')
        plt.plot(goal[0], goal[1], 'ro', markersize=8, label='目标点')
        
        if path:
            path_x = [point[0] for point in path]
            path_y = [point[1] for point in path]
            plt.plot(path_x, path_y, 'g-', linewidth=3, label='路径')
        
        plt.xlim(0, 10)
        plt.ylim(0, 10)
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.title(f'{config["title"]} (步长={config["step_size"]})')
        plt.axis('equal')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    print("RRT算法示例程序")
    print("请选择要运行的示例:")
    print("1. 简单示例（无障碍物）")
    print("2. 障碍物示例")
    print("3. 迷宫示例")
    print("4. 参数对比示例")
    print("5. 运行所有示例")
    
    choice = input("请输入选择 (1-5): ").strip()
    
    if choice == "1":
        simple_example()
    elif choice == "2":
        obstacle_example()
    elif choice == "3":
        maze_example()
    elif choice == "4":
        custom_parameters()
    elif choice == "5":
        simple_example()
        obstacle_example()
        maze_example()
        custom_parameters()
    else:
        print("直接运行障碍物示例...")
        obstacle_example()