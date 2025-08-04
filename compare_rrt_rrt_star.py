#!/usr/bin/env python3
"""
RRT vs RRT* 算法对比演示
"""

import matplotlib.pyplot as plt
from rrt import RRT, setup_chinese_font
from rrt_star import RRTStar
import time

def compare_algorithms():
    """对比RRT和RRT*算法的性能和路径质量"""
    setup_chinese_font()
    
    print("=" * 60)
    print("🚀 RRT vs RRT* 算法对比")
    print("=" * 60)
    
    # 相同的测试环境
    start = (1, 1)
    goal = (9, 9)
    obstacles = [
        (3, 3, 1.2),
        (6, 4, 1.0),
        (4, 7, 1.0),
        (7, 6, 0.8)
    ]
    
    # 算法参数
    common_params = {
        'start': start,
        'goal': goal,
        'obstacles': obstacles,
        'map_size': (10, 10),
        'step_size': 0.4,
        'max_iter': 1500
    }
    
    # 创建并运行RRT算法
    print("🔄 运行RRT算法...")
    start_time = time.time()
    rrt = RRT(**common_params)
    rrt_path, rrt_nodes = rrt.plan()
    rrt_time = time.time() - start_time
    
    # 计算RRT路径成本
    rrt_cost = 0
    if rrt_path:
        for i in range(1, len(rrt_path)):
            dx = rrt_path[i][0] - rrt_path[i-1][0]
            dy = rrt_path[i][1] - rrt_path[i-1][1]
            rrt_cost += (dx**2 + dy**2)**0.5
    
    # 创建并运行RRT*算法
    print("🔄 运行RRT*算法...")
    start_time = time.time()
    rrt_star = RRTStar(search_radius=1.2, **common_params)
    rrt_star_path, rrt_star_nodes, rrt_star_cost = rrt_star.plan()
    rrt_star_time = time.time() - start_time
    
    # 对比结果
    print("\n" + "="*60)
    print("📊 对比结果")
    print("="*60)
    
    print(f"{'指标':<15} {'RRT':<15} {'RRT*':<15} {'改进':<15}")
    print("-" * 60)
    
    if rrt_path and rrt_star_path:
        improvement = ((rrt_cost - rrt_star_cost) / rrt_cost * 100) if rrt_cost > 0 else 0
        print(f"{'路径成本':<15} {rrt_cost:<15.2f} {rrt_star_cost:<15.2f} {improvement:<15.1f}%")
    else:
        print(f"{'路径成本':<15} {'未找到':<15} {rrt_star_cost:<15.2f} {'N/A':<15}")
    
    print(f"{'路径长度':<15} {len(rrt_path) if rrt_path else 0:<15} {len(rrt_star_path) if rrt_star_path else 0:<15} {'N/A':<15}")
    print(f"{'节点数量':<15} {len(rrt_nodes):<15} {len(rrt_star_nodes):<15} {'N/A':<15}")
    print(f"{'运行时间(秒)':<15} {rrt_time:<15.2f} {rrt_star_time:<15.2f} {rrt_star_time/rrt_time:<15.2f}x")
    
    # 可视化对比
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 8))
    
    # 绘制RRT结果
    ax1.set_title('RRT 算法', fontsize=16, weight='bold')
    plot_result(ax1, rrt, rrt_path, rrt_nodes, rrt_cost, obstacles)
    
    # 绘制RRT*结果
    ax2.set_title('RRT* 算法', fontsize=16, weight='bold')
    plot_result(ax2, rrt_star, rrt_star_path, rrt_star_nodes, rrt_star_cost, obstacles)
    
    plt.tight_layout()
    plt.show()
    
    return rrt_cost, rrt_star_cost

def plot_result(ax, algorithm, path, nodes, cost, obstacles):
    """绘制单个算法的结果"""
    # 绘制障碍物
    for obstacle in obstacles:
        obs_x, obs_y, obs_radius = obstacle
        circle = plt.Circle((obs_x, obs_y), obs_radius, color='red', alpha=0.7)
        ax.add_patch(circle)
    
    # 绘制树
    for node in nodes:
        if hasattr(node, 'parent') and node.parent:
            ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 
                   'b-', alpha=0.3, linewidth=0.5)
    
    # 绘制节点
    x_coords = [node.x for node in nodes]
    y_coords = [node.y for node in nodes]
    ax.plot(x_coords, y_coords, 'bo', markersize=1.5, alpha=0.6)
    
    # 绘制起点和终点
    ax.plot(algorithm.start.x, algorithm.start.y, 'go', markersize=10, label='起点')
    ax.plot(algorithm.goal.x, algorithm.goal.y, 'ro', markersize=10, label='目标点')
    
    # 绘制路径
    if path:
        path_x = [point[0] for point in path]
        path_y = [point[1] for point in path]
        ax.plot(path_x, path_y, 'g-', linewidth=4, 
               label=f'路径 (成本: {cost:.2f})', alpha=0.8)
    
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_xlabel('X坐标')
    ax.set_ylabel('Y坐标')
    
    # 添加统计信息
    info_text = f"""统计信息:
• 节点数: {len(nodes)}
• 路径点数: {len(path) if path else 0}
• 路径成本: {cost:.2f}"""
    
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes, 
           fontsize=10, verticalalignment='top',
           bbox=dict(boxstyle="round,pad=0.5", facecolor="lightblue", alpha=0.8))

def explain_differences():
    """详细解释RRT和RRT*的区别"""
    print("\n" + "="*60)
    print("📚 RRT vs RRT* 详细对比")
    print("="*60)
    
    differences = [
        {
            "方面": "算法目标",
            "RRT": "快速探索，找到可行路径",
            "RRT*": "渐近最优，找到接近最优的路径"
        },
        {
            "方面": "路径质量",
            "RRT": "通常较差，路径可能很曲折",
            "RRT*": "逐渐优化，最终接近最优路径"
        },
        {
            "方面": "计算复杂度",
            "RRT": "O(n) - 线性复杂度",
            "RRT*": "O(n log n) - 对数线性复杂度"
        },
        {
            "方面": "收敛性",
            "RRT": "概率完备，但不保证最优性",
            "RRT*": "概率完备且渐近最优"
        },
        {
            "方面": "运行时间",
            "RRT": "较快",
            "RRT*": "较慢（因为重新布线操作）"
        }
    ]
    
    print(f"{'方面':<12} {'RRT':<25} {'RRT*':<30}")
    print("-" * 67)
    for diff in differences:
        print(f"{diff['方面']:<12} {diff['RRT']:<25} {diff['RRT*']:<30}")
    
    print("\n🔧 RRT*的关键改进:")
    print("1. 🎯 邻近节点搜索 - 在固定半径内寻找多个候选父节点")
    print("2. 🏆 最优父节点选择 - 选择成本最低的路径")
    print("3. 🔄 重新布线 - 动态优化已有节点的连接")
    print("4. 📈 成本传播 - 递归更新子树的路径成本")

def algorithm_steps_comparison():
    """对比算法步骤"""
    print("\n" + "="*60)
    print("⚙️  算法步骤对比")
    print("="*60)
    
    print("🔵 RRT 算法步骤:")
    rrt_steps = [
        "1. 随机采样点",
        "2. 找最近邻节点",
        "3. 向采样点扩展",
        "4. 碰撞检测",
        "5. 添加新节点",
        "6. 检查是否到达目标"
    ]
    
    for step in rrt_steps:
        print(f"   {step}")
    
    print("\n🟢 RRT* 算法步骤:")
    rrt_star_steps = [
        "1. 随机采样点",
        "2. 找最近邻节点", 
        "3. 向采样点扩展",
        "4. 🆕 寻找邻近节点集合",
        "5. 🆕 选择最优父节点",
        "6. 添加新节点",
        "7. 🆕 重新布线邻近节点",
        "8. 检查是否到达目标"
    ]
    
    for step in rrt_star_steps:
        marker = "🆕 " if "🆕" in step else "   "
        print(f"{marker}{step}")
    
    print("\n💡 关键差异:")
    print("   • RRT*增加了3个关键步骤（4、5、7）")
    print("   • 这些步骤确保了路径的逐步优化")
    print("   • 代价是增加了计算复杂度")

if __name__ == "__main__":
    # 运行对比
    rrt_cost, rrt_star_cost = compare_algorithms()
    
    # 解释差异
    explain_differences()
    algorithm_steps_comparison()
    
    print("\n" + "="*60)
    print("🎯 总结")
    print("="*60)
    print("选择建议:")
    print("• 需要快速规划 → 选择 RRT")
    print("• 需要高质量路径 → 选择 RRT*") 
    print("• 实时应用 → 选择 RRT")
    print("• 离线规划 → 选择 RRT*")
    print("• 路径成本敏感 → 选择 RRT*")
    print("• 计算资源有限 → 选择 RRT")