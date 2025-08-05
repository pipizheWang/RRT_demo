#!/usr/bin/env python3
"""
SCP轨迹规划运行脚本
集成RRT/RRT*路径规划结果，使用SCP方法生成优化的轨迹
"""

from rrt import RRT
from rrt_star import RRTStar
from scp_trajectory import SCPTrajectoryPlanner
import matplotlib.pyplot as plt
import numpy as np
import time

def run_rrt_to_scp():
    """使用RRT进行路径规划，然后用SCP优化轨迹"""
    print("🌲➡️⚡ RRT + SCP 集成轨迹规划")
    print("=" * 50)
    
    # 1. 设置环境参数
    start = (1, 1)
    goal = (9, 9)
    obstacles = [
        (3, 3, 1.2),
        (6, 4, 1.0),
        (4, 7, 1.0),
        (7, 6, 0.8)
    ]
    
    print(f"环境: 起点{start} -> 终点{goal}, {len(obstacles)}个障碍物")
    
    # 2. RRT路径规划
    
    rrt = RRT(
        start=start,
        goal=goal,
        obstacles=obstacles,
        map_size=(10, 10),
        step_size=0.5,
        max_iter=2000
    )
    
    start_time = time.time()
    path, nodes = rrt.plan()
    rrt_time = time.time() - start_time
    
    if not path:
        print("❌ RRT路径规划失败")
        return
    
    rrt_path_length = calculate_path_length(path)
    print(f"✅ RRT: 长度{rrt_path_length:.2f}m, {len(path)}点, 用时{rrt_time:.2f}s")
    
    # 3. SCP轨迹优化
    
    # 根据路径长度自动调整时间跨度
    path_length = calculate_path_length(path)
    estimated_time = path_length / 1.2  # 期望平均速度1.2 m/s
    estimated_time = max(estimated_time, 5.0)  # 最少5秒
    estimated_time = min(estimated_time, 15.0)  # 最多15秒
    
    scp_planner = SCPTrajectoryPlanner(
        max_velocity=2.0,        # 恢复合理的速度约束
        max_acceleration=1.5,    # 恢复合理的加速度约束  
        max_curvature=0.5,
        obstacles=obstacles,
        time_horizon=estimated_time,  # 根据路径长度自动调整
        num_segments=max(30, int(estimated_time * 3))  # 每秒3-4个segment
    )
    
    scp_planner.set_path_waypoints(path)
    
    start_time = time.time()
    trajectory, success = scp_planner.plan_trajectory(max_iterations=5, use_staged_optimization=True)
    scp_time = time.time() - start_time
    
    if not success:
        print("❌ SCP轨迹优化失败")
        return
    
    # 4. 结果分析
    scp_path_length = calculate_trajectory_length(trajectory)
    print(f"✅ SCP: 长度{scp_path_length:.2f}m, 用时{scp_time:.2f}s")
    print(f"总计算时间: {rrt_time + scp_time:.2f}s")
    
    # 可视化对比
    visualize_comparison(path, trajectory, obstacles, scp_planner)
    
    # 显示轨迹详细信息
    scp_planner.plot_trajectory_profiles()

def run_rrt_star_to_scp():
    """使用RRT*进行路径规划，然后用SCP优化轨迹"""
    print("🌟➡️⚡ RRT* + SCP 集成轨迹规划")
    print("=" * 50)
    
    # 1. 设置环境参数  
    start = (1, 1)
    goal = (9, 9)
    obstacles = [
        (3, 3, 1.2),
        (6, 4, 1.0),
        (4, 7, 1.0),
        (7, 6, 0.8)
    ]
    
    print(f"环境: 起点{start} -> 终点{goal}, {len(obstacles)}个障碍物")
    
    # 2. RRT*路径规划
    
    rrt_star = RRTStar(
        start=start,
        goal=goal,
        obstacles=obstacles,
        map_size=(10, 10),
        step_size=0.4,
        search_radius=1.2,
        max_iter=2000
    )
    
    start_time = time.time()
    path, nodes, path_cost = rrt_star.plan()
    rrt_star_time = time.time() - start_time
    
    if not path:
        print("❌ RRT*路径规划失败")
        return
    
    print(f"✅ RRT*: 成本{path_cost:.2f}, {len(path)}点, 用时{rrt_star_time:.2f}s")
    
    # 3. SCP轨迹优化
    
    # 根据路径长度自动调整时间跨度  
    path_length = path_cost  # RRT*已经计算了路径成本
    estimated_time = path_length / 1.0  # 期望平均速度1.0 m/s（RRT*路径质量更高）
    estimated_time = max(estimated_time, 6.0)  # 最少6秒
    estimated_time = min(estimated_time, 12.0)  # 最多12秒
    
    scp_planner = SCPTrajectoryPlanner(
        max_velocity=1.8,        # 适中的速度设置
        max_acceleration=1.2,    # 适中的加速度设置
        max_curvature=0.4,
        obstacles=obstacles,
        time_horizon=estimated_time,  # 根据路径长度自动调整
        num_segments=max(25, int(estimated_time * 3))  # 每秒3个segment
    )
    
    scp_planner.set_path_waypoints(path)
    
    start_time = time.time()
    trajectory, success = scp_planner.plan_trajectory(max_iterations=5, use_staged_optimization=True)
    scp_time = time.time() - start_time
    
    if not success:
        print("❌ SCP轨迹优化失败")
        return
    
    # 4. 结果分析
    scp_path_length = calculate_trajectory_length(trajectory)
    print(f"✅ SCP: 长度{scp_path_length:.2f}m, 用时{scp_time:.2f}s") 
    print(f"总计算时间: {rrt_star_time + scp_time:.2f}s")
    
    # 可视化对比
    visualize_comparison(path, trajectory, obstacles, scp_planner)
    
    # 显示轨迹详细信息
    scp_planner.plot_trajectory_profiles()

def compare_all_methods():
    """对比RRT、RRT*、RRT+SCP、RRT*+SCP四种方法"""
    print("📊 完整算法对比分析")
    print("=" * 50)
    
    # 环境设置
    start = (1, 1)
    goal = (9, 9)
    obstacles = [
        (3, 3, 1.2),
        (6, 4, 1.0), 
        (4, 7, 1.0),
        (7, 6, 0.8)
    ]
    
    results = {}
    
    # 1. RRT
    print("\n测试 RRT...")
    rrt = RRT(start=start, goal=goal, obstacles=obstacles, 
              map_size=(10, 10), step_size=0.5, max_iter=2000)
    
    start_time = time.time()
    rrt_path, rrt_nodes = rrt.plan()
    rrt_time = time.time() - start_time
    
    if rrt_path:
        results['RRT'] = {
            'path': rrt_path,
            'length': calculate_path_length(rrt_path),
            'time': rrt_time,
            'nodes': len(rrt_nodes)
        }
        print(f"✅ RRT: 长度={results['RRT']['length']:.2f}m, 时间={rrt_time:.2f}s")
    
    # 2. RRT*
    print("测试 RRT*...")
    rrt_star = RRTStar(start=start, goal=goal, obstacles=obstacles,
                       map_size=(10, 10), step_size=0.4, 
                       search_radius=1.2, max_iter=2000)
    
    start_time = time.time()
    rrt_star_path, rrt_star_nodes, rrt_star_cost = rrt_star.plan()
    rrt_star_time = time.time() - start_time
    
    if rrt_star_path:
        results['RRT*'] = {
            'path': rrt_star_path,
            'length': rrt_star_cost,
            'time': rrt_star_time,
            'nodes': len(rrt_star_nodes)
        }
        print(f"✅ RRT*: 长度={rrt_star_cost:.2f}m, 时间={rrt_star_time:.2f}s")
    
    # 3. RRT + SCP
    if 'RRT' in results:
        print("运行 RRT+SCP...")
        rrt_path_length = results['RRT']['length']
        estimated_time1 = max(5.0, min(15.0, rrt_path_length / 1.2))
        
        scp1 = SCPTrajectoryPlanner(max_velocity=2.0, max_acceleration=1.5,
                                   obstacles=obstacles, time_horizon=estimated_time1, 
                                   num_segments=max(30, int(estimated_time1 * 3)))
        scp1.set_path_waypoints(results['RRT']['path'])
        
        start_time = time.time()
        traj1, success1 = scp1.plan_trajectory(max_iterations=4, use_staged_optimization=True)
        scp1_time = time.time() - start_time
        
        if success1:
            results['RRT+SCP'] = {
                'trajectory': traj1,
                'length': calculate_trajectory_length(traj1),
                'time': results['RRT']['time'] + scp1_time,
                'planning_time': results['RRT']['time'],
                'optimization_time': scp1_time
            }
            print(f"✅ RRT+SCP: 长度{results['RRT+SCP']['length']:.2f}m, 总时间{results['RRT+SCP']['time']:.2f}s")
    
    # 4. RRT* + SCP
    if 'RRT*' in results:
        print("运行 RRT*+SCP...")
        rrt_star_path_length = results['RRT*']['length']
        estimated_time2 = max(6.0, min(12.0, rrt_star_path_length / 1.0))
        
        scp2 = SCPTrajectoryPlanner(max_velocity=1.8, max_acceleration=1.2,
                                   obstacles=obstacles, time_horizon=estimated_time2,
                                   num_segments=max(25, int(estimated_time2 * 3)))
        scp2.set_path_waypoints(results['RRT*']['path'])
        
        start_time = time.time()
        traj2, success2 = scp2.plan_trajectory(max_iterations=4, use_staged_optimization=True)
        scp2_time = time.time() - start_time
        
        if success2:
            results['RRT*+SCP'] = {
                'trajectory': traj2,
                'length': calculate_trajectory_length(traj2),
                'time': results['RRT*']['time'] + scp2_time,
                'planning_time': results['RRT*']['time'],
                'optimization_time': scp2_time
            }
            print(f"✅ RRT*+SCP: 长度{results['RRT*+SCP']['length']:.2f}m, 总时间{results['RRT*+SCP']['time']:.2f}s")
    
    # 结果总结
    print(f"\n📊 算法性能对比")
    print(f"{'算法':<12} {'长度(m)':<10} {'时间(s)':<10} {'类型'}")
    print("-" * 45)
    
    for method, data in results.items():
        path_type = '连续轨迹' if 'SCP' in method else '离散路径'
        print(f"{method:<12} {data['length']:<10.2f} {data['time']:<10.2f} {path_type}")
    
    # 可视化所有结果
    visualize_all_comparison(results, obstacles)

def calculate_path_length(path):
    """计算路径长度"""
    if not path or len(path) < 2:
        return 0.0
    
    length = 0.0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i-1][0]
        dy = path[i][1] - path[i-1][1]
        length += np.sqrt(dx*dx + dy*dy)
    
    return length

def calculate_trajectory_length(trajectory):
    """计算轨迹长度"""
    if trajectory is None or len(trajectory) < 2:
        return 0.0
    
    length = 0.0
    for i in range(1, len(trajectory)):
        dx = trajectory[i, 0] - trajectory[i-1, 0]
        dy = trajectory[i, 1] - trajectory[i-1, 1]
        length += np.sqrt(dx*dx + dy*dy)
    
    return length

def visualize_comparison(original_path, optimized_trajectory, obstacles, scp_planner):
    """可视化原始路径和优化轨迹的对比"""
    plt.figure(figsize=(14, 6))
    
    # 左图：原始路径
    plt.subplot(1, 2, 1)
    
    # 绘制障碍物
    for obstacle in obstacles:
        obs_x, obs_y, obs_radius = obstacle
        circle = plt.Circle((obs_x, obs_y), obs_radius, color='red', alpha=0.7)
        plt.gca().add_patch(circle)
    
    # 绘制原始路径
    path_array = np.array(original_path)
    plt.plot(path_array[:, 0], path_array[:, 1], 'ro-', 
            markersize=6, linewidth=2, label='RRT/RRT*路径')
    plt.plot(path_array[0, 0], path_array[0, 1], 'go', markersize=10, label='起点')
    plt.plot(path_array[-1, 0], path_array[-1, 1], 'bs', markersize=10, label='终点')
    
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.xlabel('X 坐标 (m)')
    plt.ylabel('Y 坐标 (m)')
    plt.title('原始路径规划结果')
    plt.legend()
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    
    # 右图：优化轨迹
    plt.subplot(1, 2, 2)
    
    # 绘制障碍物
    for obstacle in obstacles:
        obs_x, obs_y, obs_radius = obstacle
        circle = plt.Circle((obs_x, obs_y), obs_radius, color='red', alpha=0.7)
        plt.gca().add_patch(circle)
    
    # 绘制原始waypoints（淡色）
    plt.plot(path_array[:, 0], path_array[:, 1], 'ro-', 
            markersize=4, linewidth=1, alpha=0.4, label='原始waypoints')
    
    # 绘制优化轨迹
    positions = optimized_trajectory[:, :2]
    plt.plot(positions[:, 0], positions[:, 1], 'g-', 
            linewidth=3, label='SCP优化轨迹')
    plt.plot(positions[0, 0], positions[0, 1], 'go', markersize=10, label='起点')
    plt.plot(positions[-1, 0], positions[-1, 1], 'bs', markersize=10, label='终点')
    
    # 绘制部分速度向量
    step = len(optimized_trajectory) // 15
    for i in range(0, len(optimized_trajectory), step):
        pos = optimized_trajectory[i, :2]
        vel = optimized_trajectory[i, 2:4]
        scale = 0.2
        plt.arrow(pos[0], pos[1], vel[0]*scale, vel[1]*scale,
                 head_width=0.08, head_length=0.08, fc='blue', 
                 ec='blue', alpha=0.6)
    
    plt.grid(True, alpha=0.3)
    plt.axis('equal')  
    plt.xlabel('X 坐标 (m)')
    plt.ylabel('Y 坐标 (m)')
    plt.title('SCP轨迹优化结果')
    plt.legend()
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    
    plt.tight_layout()
    plt.show()

def visualize_all_comparison(results, obstacles):
    """可视化所有算法的对比结果"""
    plt.figure(figsize=(16, 8))
    
    colors = {'RRT': 'blue', 'RRT*': 'orange', 'RRT+SCP': 'green', 'RRT*+SCP': 'purple'}
    
    # 左图：路径/轨迹对比
    plt.subplot(1, 2, 1)
    
    # 绘制障碍物
    for obstacle in obstacles:
        obs_x, obs_y, obs_radius = obstacle
        circle = plt.Circle((obs_x, obs_y), obs_radius, color='red', alpha=0.7)
        plt.gca().add_patch(circle)
    
    # 绘制各算法结果
    for method, data in results.items():
        color = colors.get(method, 'black')
        
        if 'SCP' in method:
            # 绘制连续轨迹
            positions = data['trajectory'][:, :2]
            plt.plot(positions[:, 0], positions[:, 1], '-', 
                    color=color, linewidth=3, alpha=0.8, label=method)
        else:
            # 绘制离散路径
            path_array = np.array(data['path'])
            plt.plot(path_array[:, 0], path_array[:, 1], 'o-', 
                    color=color, markersize=4, linewidth=2, alpha=0.7, label=method)
    
    # 标记起点和终点
    plt.plot(1, 1, 'go', markersize=12, label='起点')
    plt.plot(9, 9, 'rs', markersize=12, label='终点')
    
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.xlabel('X 坐标 (m)')
    plt.ylabel('Y 坐标 (m)')
    plt.title('所有算法路径/轨迹对比')
    plt.legend()
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    
    # 右图：性能对比条形图
    plt.subplot(1, 2, 2)
    
    methods = list(results.keys())
    lengths = [results[m]['length'] for m in methods]
    times = [results[m]['time'] for m in methods]
    
    x = np.arange(len(methods))
    width = 0.35
    
    # 双y轴图
    ax1 = plt.gca()
    bars1 = ax1.bar(x - width/2, lengths, width, label='路径长度 (m)', 
                    color=[colors.get(m, 'gray') for m in methods], alpha=0.7)
    ax1.set_xlabel('算法')
    ax1.set_ylabel('路径长度 (m)', color='blue')
    ax1.tick_params(axis='y', labelcolor='blue')
    
    ax2 = ax1.twinx()
    bars2 = ax2.bar(x + width/2, times, width, label='计算时间 (s)', 
                    color='red', alpha=0.5)
    ax2.set_ylabel('计算时间 (s)', color='red')
    ax2.tick_params(axis='y', labelcolor='red')
    
    ax1.set_xticks(x)
    ax1.set_xticklabels(methods)
    ax1.set_title('算法性能对比')
    
    # 添加数值标签
    for i, (bar1, bar2) in enumerate(zip(bars1, bars2)):
        height1 = bar1.get_height()
        height2 = bar2.get_height()
        ax1.text(bar1.get_x() + bar1.get_width()/2., height1 + 0.1,
                f'{height1:.1f}', ha='center', va='bottom', fontsize=9)
        ax2.text(bar2.get_x() + bar2.get_width()/2., height2 + 0.02,
                f'{height2:.2f}', ha='center', va='bottom', fontsize=9)
    
    # 图例
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')
    
    plt.tight_layout()
    plt.show()

def custom_scp_demo():
    """自定义参数的SCP演示"""
    print("🛠️ 自定义SCP参数演示")
    
    # 简化的用户输入
    print("选择基础算法: 1-RRT, 2-RRT*")
    choice = input("请选择 (默认1): ").strip() or "1"
    
    obstacles = [(3, 3, 1.0), (6, 6, 0.8)]
    start, goal = (1, 1), (9, 9)
    
    # 路径规划
    if choice == "2":
        planner = RRTStar(start=start, goal=goal, obstacles=obstacles,
                         map_size=(10, 10), step_size=0.4, search_radius=1.2, max_iter=1500)
        path, _, _ = planner.plan()
        print("使用RRT*规划路径")
    else:
        planner = RRT(start=start, goal=goal, obstacles=obstacles,
                     map_size=(10, 10), step_size=0.5, max_iter=1500)
        path, _ = planner.plan()
        print("使用RRT规划路径")
    
    if not path:
        print("❌ 路径规划失败")
        return
    
    # SCP轨迹优化
    path_length = calculate_path_length(path)
    estimated_time = max(6.0, path_length / 1.5)
    
    scp_planner = SCPTrajectoryPlanner(
        max_velocity=2.0,
        max_acceleration=1.5,
        obstacles=obstacles,
        time_horizon=estimated_time,
        num_segments=max(25, int(estimated_time * 3))
    )
    
    scp_planner.set_path_waypoints(path)
    trajectory, success = scp_planner.plan_trajectory(max_iterations=4)
    
    if success:
        print("✅ SCP优化完成")
        visualize_comparison(path, trajectory, obstacles, scp_planner)
    else:
        print("❌ SCP优化失败")

if __name__ == "__main__":
    print("🚀 SCP轨迹规划集成系统")
    print("1. RRT + SCP")
    print("2. RRT* + SCP") 
    print("3. 完整算法对比")
    print("4. 自定义演示")
    
    choice = input("选择模式 (1-4): ").strip()
    
    if choice == "1":
        run_rrt_to_scp()
    elif choice == "2":
        run_rrt_star_to_scp()
    elif choice == "3":
        compare_all_methods()
    elif choice == "4":
        custom_scp_demo()
    else:
        print("运行完整对比...")
        compare_all_methods()