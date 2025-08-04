#!/usr/bin/env python3
"""
RRT 算法专用运行脚本
快速运行RRT路径规划算法
"""

from rrt import RRT, setup_chinese_font
import matplotlib.pyplot as plt
import math

def calculate_path_cost(path):
    """计算路径的实际成本"""
    if not path or len(path) < 2:
        return 0.0
    
    total_cost = 0.0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i-1][0]
        dy = path[i][1] - path[i-1][1]
        segment_cost = math.sqrt(dx*dx + dy*dy)
        total_cost += segment_cost
    
    return total_cost

def simple_rrt():
    """简单的RRT示例"""
    print("🌲 RRT 算法演示")
    print("=" * 40)
    
    # 设置起点和终点
    start = (1, 1)
    goal = (9, 9)
    
    # 定义障碍物
    obstacles = [
        (3, 3, 1.2),
        (6, 4, 1.0),
        (4, 7, 1.0),
        (7, 6, 0.8)
    ]
    
    print(f"起点: {start}")
    print(f"终点: {goal}")
    print(f"障碍物数量: {len(obstacles)}")
    
    # 创建RRT实例
    rrt = RRT(
        start=start, 
        goal=goal, 
        obstacles=obstacles, 
        map_size=(10, 10), 
        step_size=0.5,          # RRT的标准步长
        max_iter=1500          # 适中的迭代次数
    )
    
    print("\n开始RRT路径规划...")
    path, nodes = rrt.plan()
    
    if path:
        path_cost = calculate_path_cost(path)
        print(f"✅ 路径规划成功!")
        print(f"   路径成本: {path_cost:.2f}")
        print(f"   路径点数: {len(path)}")
        print(f"   总节点数: {len(nodes)}")
        print(f"   步长设置: {rrt.step_size}")
        
        # 可视化结果
        print("\n正在显示结果...")
        rrt.visualize(path)
        
    else:
        print("❌ 未找到路径，请尝试:")
        print("   - 增加迭代次数")
        print("   - 调整步长大小")
        print("   - 简化障碍物环境")

def custom_rrt():
    """自定义参数的RRT示例"""
    print("🛠️  自定义RRT参数")
    print("=" * 40)
    
    # 让用户输入参数
    try:
        print("请输入起点坐标 (默认: 1,1):")
        start_input = input("格式: x,y > ").strip()
        if start_input:
            start = tuple(map(float, start_input.split(',')))
        else:
            start = (1, 1)
        
        print("请输入终点坐标 (默认: 9,9):")
        goal_input = input("格式: x,y > ").strip()
        if goal_input:
            goal = tuple(map(float, goal_input.split(',')))
        else:
            goal = (9, 9)
        
        print("请输入步长 (默认: 0.5):")
        step_input = input("步长 > ").strip()
        step_size = float(step_input) if step_input else 0.5
        
        print("请输入最大迭代次数 (默认: 1500):")
        iter_input = input("次数 > ").strip()
        max_iter = int(iter_input) if iter_input else 1500
        
    except ValueError:
        print("输入格式错误，使用默认参数")
        start = (1, 1)
        goal = (9, 9)
        step_size = 0.5
        max_iter = 1500
    
    # 默认障碍物
    obstacles = [(4, 4, 1.0), (6, 6, 0.8)]
    
    # 运行RRT
    rrt = RRT(
        start=start, 
        goal=goal, 
        obstacles=obstacles, 
        map_size=(10, 10), 
        step_size=step_size,
        max_iter=max_iter
    )
    
    path, nodes = rrt.plan()
    
    if path:
        path_cost = calculate_path_cost(path)
        print(f"✅ 成功! 路径成本: {path_cost:.2f}")
        rrt.visualize(path)
    else:
        print("❌ 未找到路径")

def compare_different_step_size():
    """对比不同步长的RRT效果"""
    print("📊 对比不同步长的RRT效果")
    print("=" * 40)
    
    start = (1, 1)
    goal = (8, 8)
    obstacles = [(4, 4, 1.5)]
    
    # 不同的步长
    step_sizes = [0.3, 0.6, 1.2]
    
    setup_chinese_font()
    
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    
    for i, step_size in enumerate(step_sizes):
        print(f"\n测试步长: {step_size}")
        
        rrt = RRT(
            start=start, 
            goal=goal, 
            obstacles=obstacles, 
            map_size=(10, 10), 
            step_size=step_size,
            max_iter=1000
        )
        
        path, nodes = rrt.plan()
        path_cost = calculate_path_cost(path) if path else float('inf')
        
        # 绘制到子图
        ax = axes[i]
        
        # 绘制障碍物
        for obstacle in obstacles:
            obs_x, obs_y, obs_radius = obstacle
            circle = plt.Circle((obs_x, obs_y), obs_radius, color='red', alpha=0.7)
            ax.add_patch(circle)
        
        # 绘制树
        for node in nodes:
            if node.parent:
                ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 
                       'b-', alpha=0.3, linewidth=0.5)
        
        # 绘制路径
        if path:
            path_x = [point[0] for point in path]
            path_y = [point[1] for point in path]
            ax.plot(path_x, path_y, 'g-', linewidth=3, alpha=0.8)
        
        ax.plot(start[0], start[1], 'go', markersize=8)
        ax.plot(goal[0], goal[1], 'ro', markersize=8)
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.grid(True, alpha=0.3)
        ax.set_title(f'步长={step_size}\n成本={path_cost:.2f}' if path else f'步长={step_size}\n未找到路径')
    
    plt.tight_layout()
    plt.show()

def maze_rrt():
    """迷宫环境的RRT示例"""
    print("🏰 迷宫环境RRT挑战")
    print("=" * 40)
    
    start = (0.5, 0.5)
    goal = (9.5, 9.5)
    
    # 创建迷宫障碍物
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
    
    print(f"起点: {start}")
    print(f"终点: {goal}")
    print(f"迷宫障碍物数量: {len(obstacles)}")
    
    # 对复杂环境使用小步长
    rrt = RRT(
        start=start, 
        goal=goal, 
        obstacles=obstacles, 
        map_size=(10, 10), 
        step_size=0.2,      # 小步长适应复杂环境
        max_iter=3000       # 更多迭代次数
    )
    
    print("\n开始迷宫路径规划...")
    path, nodes = rrt.plan()
    
    if path:
        path_cost = calculate_path_cost(path)
        print(f"✅ 迷宫挑战成功!")
        print(f"   路径成本: {path_cost:.2f}")
        print(f"   路径点数: {len(path)}")
        print(f"   搜索节点数: {len(nodes)}")
        
        rrt.visualize(path)
    else:
        print("❌ 迷宫太复杂，未找到路径")
        print("建议：减小步长或增加迭代次数")

def performance_test():
    """RRT性能测试"""
    print("⚡ RRT算法性能测试")
    print("=" * 40)
    
    import time
    
    start = (1, 1)
    goal = (9, 9)
    obstacles = [(3, 3, 1), (6, 6, 1), (5, 2, 0.8)]
    
    test_configs = [
        {"name": "快速模式", "step_size": 0.8, "max_iter": 500},
        {"name": "标准模式", "step_size": 0.5, "max_iter": 1500},
        {"name": "精细模式", "step_size": 0.2, "max_iter": 3000},
    ]
    
    print("测试配置对比:")
    print(f"{'模式':<10} {'时间(秒)':<10} {'路径成本':<10} {'节点数':<10}")
    print("-" * 45)
    
    for config in test_configs:
        start_time = time.time()
        
        rrt = RRT(
            start=start, 
            goal=goal, 
            obstacles=obstacles, 
            map_size=(10, 10), 
            step_size=config["step_size"],
            max_iter=config["max_iter"]
        )
        
        path, nodes = rrt.plan()
        elapsed_time = time.time() - start_time
        
        if path:
            path_cost = calculate_path_cost(path)
            print(f"{config['name']:<10} {elapsed_time:<10.3f} {path_cost:<10.2f} {len(nodes):<10}")
        else:
            print(f"{config['name']:<10} {elapsed_time:<10.3f} {'未找到':<10} {len(nodes):<10}")

if __name__ == "__main__":
    print("RRT 算法运行脚本")
    print("选择运行模式:")
    print("1. 简单示例")
    print("2. 自定义参数")
    print("3. 步长对比")
    print("4. 迷宫挑战")
    print("5. 性能测试")
    
    choice = input("请选择 (1-5): ").strip()
    
    if choice == "1":
        simple_rrt()
    elif choice == "2":
        custom_rrt()
    elif choice == "3":
        compare_different_step_size()
    elif choice == "4":
        maze_rrt()
    elif choice == "5":
        performance_test()
    else:
        print("默认运行简单示例...")
        simple_rrt()