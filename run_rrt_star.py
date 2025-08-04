#!/usr/bin/env python3
"""
RRT* 算法专用运行脚本
快速运行RRT*路径规划算法
"""

from rrt_star import RRTStar, setup_chinese_font

def simple_rrt_star():
    """简单的RRT*示例"""
    print("🌟 RRT* 算法演示")
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
    
    # 创建RRT*实例
    rrt_star = RRTStar(
        start=start, 
        goal=goal, 
        obstacles=obstacles, 
        map_size=(10, 10), 
        step_size=0.4,           # 较小的步长以获得更好的路径
        search_radius=1.2,       # 搜索半径
        max_iter=2000           # 足够的迭代次数
    )
    
    print("\n开始RRT*路径规划...")
    path, nodes, path_cost = rrt_star.plan()
    
    if path:
        print(f"✅ 路径规划成功!")
        print(f"   路径成本: {path_cost:.2f}")
        print(f"   路径点数: {len(path)}")
        print(f"   总节点数: {len(nodes)}")
        print(f"   搜索半径: {rrt_star.search_radius}")
        
        # 可视化结果
        print("\n正在显示结果...")
        rrt_star.visualize(path, path_cost)
        
    else:
        print("❌ 未找到路径，请尝试:")
        print("   - 增加迭代次数")
        print("   - 调整搜索半径")
        print("   - 减小步长")

def custom_rrt_star():
    """自定义参数的RRT*示例"""
    print("🛠️  自定义RRT*参数")
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
        
        print("请输入搜索半径 (默认: 1.2):")
        radius_input = input("半径 > ").strip()
        search_radius = float(radius_input) if radius_input else 1.2
        
        print("请输入最大迭代次数 (默认: 2000):")
        iter_input = input("次数 > ").strip()
        max_iter = int(iter_input) if iter_input else 2000
        
    except ValueError:
        print("输入格式错误，使用默认参数")
        start = (1, 1)
        goal = (9, 9)
        search_radius = 1.2
        max_iter = 2000
    
    # 默认障碍物
    obstacles = [(4, 4, 1.0), (6, 6, 0.8)]
    
    # 运行RRT*
    rrt_star = RRTStar(
        start=start, 
        goal=goal, 
        obstacles=obstacles, 
        map_size=(10, 10), 
        step_size=0.5,
        search_radius=search_radius,
        max_iter=max_iter
    )
    
    path, nodes, path_cost = rrt_star.plan()
    
    if path:
        print(f"✅ 成功! 路径成本: {path_cost:.2f}")
        rrt_star.visualize(path, path_cost)
    else:
        print("❌ 未找到路径")

def compare_different_radius():
    """对比不同搜索半径的效果"""
    print("📊 对比不同搜索半径的RRT*效果")
    print("=" * 40)
    
    start = (1, 1)
    goal = (8, 8)
    obstacles = [(4, 4, 1.5)]
    
    # 不同的搜索半径
    radius_list = [0.8, 1.2, 2.0]
    
    import matplotlib.pyplot as plt
    setup_chinese_font()
    
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    
    for i, radius in enumerate(radius_list):
        print(f"\n测试搜索半径: {radius}")
        
        rrt_star = RRTStar(
            start=start, 
            goal=goal, 
            obstacles=obstacles, 
            map_size=(10, 10), 
            step_size=0.4,
            search_radius=radius,
            max_iter=1000
        )
        
        path, nodes, path_cost = rrt_star.plan()
        
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
        ax.set_title(f'搜索半径={radius}\n成本={path_cost:.2f}' if path else f'搜索半径={radius}\n未找到路径')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    print("RRT* 算法运行脚本")
    print("选择运行模式:")
    print("1. 简单示例")
    print("2. 自定义参数")
    print("3. 搜索半径对比")
    
    choice = input("请选择 (1-3): ").strip()
    
    if choice == "1":
        simple_rrt_star()
    elif choice == "2":
        custom_rrt_star()
    elif choice == "3":
        compare_different_radius()
    else:
        print("默认运行简单示例...")
        simple_rrt_star()