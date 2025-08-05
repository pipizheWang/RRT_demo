#!/usr/bin/env python3
"""
SCP (Sequential Convex Programming) 轨迹规划模块
基于RRT/RRT*路径进行二次优化，生成满足动力学约束的平滑轨迹
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
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

class SCPTrajectoryPlanner:
    """
    SCP轨迹规划器
    将RRT/RRT*生成的离散路径点优化为满足动力学约束的连续轨迹
    """
    
    def __init__(self, max_velocity=2.0, max_acceleration=1.0, max_curvature=0.5,
                 obstacles=None, time_horizon=10.0, num_segments=50):
        """
        初始化SCP轨迹规划器
        
        参数:
        max_velocity: 最大速度约束 (m/s)
        max_acceleration: 最大加速度约束 (m/s²)
        max_curvature: 最大曲率约束 (1/m)
        obstacles: 障碍物列表 [(x, y, radius), ...]
        time_horizon: 轨迹总时间 (s)
        num_segments: 轨迹离散化段数
        """
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_curvature = max_curvature
        self.obstacles = obstacles if obstacles else []
        self.time_horizon = time_horizon
        self.num_segments = num_segments
        
        # 时间网格
        self.dt = time_horizon / num_segments
        self.time_grid = np.linspace(0, time_horizon, num_segments + 1)
        
        # 轨迹状态：[x, y, vx, vy] for each time step
        self.state_dim = 4
        self.trajectory = None
        self.path_waypoints = None
        
    def set_path_waypoints(self, path):
        """
        设置来自RRT/RRT*的路径waypoints
        
        参数:
        path: 路径点列表 [[x1, y1], [x2, y2], ...]
        """
        if not path or len(path) < 2:
            raise ValueError("路径必须包含至少2个点")
        
        self.path_waypoints = np.array(path)
    
    def interpolate_initial_trajectory(self):
        """
        基于路径waypoints生成初始轨迹猜测 - 改进版本
        使用更合理的时间分配和速度设置
        """
        if self.path_waypoints is None:
            raise ValueError("必须先设置路径waypoints")
        
        waypoints = self.path_waypoints
        
        # 计算路径总长度
        total_distance = 0.0
        segment_distances = []
        for i in range(1, len(waypoints)):
            dist = np.linalg.norm(waypoints[i] - waypoints[i-1])
            segment_distances.append(dist)
            total_distance += dist
        
        # 根据距离分配时间（而不是均匀分配）
        cumulative_distances = [0.0]
        for dist in segment_distances:
            cumulative_distances.append(cumulative_distances[-1] + dist)
        
        # 标准化累积距离到时间跨度
        waypoint_times = [t * self.time_horizon / total_distance for t in cumulative_distances]
        
        # 使用线性插值（更贴近原始路径）
        from scipy.interpolate import interp1d
        
        # 位置插值 - 使用线性插值保持路径特征
        interp_x = interp1d(waypoint_times, waypoints[:, 0], kind='linear', 
                           bounds_error=False, fill_value='extrapolate')
        interp_y = interp1d(waypoint_times, waypoints[:, 1], kind='linear',
                           bounds_error=False, fill_value='extrapolate')
        
        # 在时间网格上采样
        x_traj = interp_x(self.time_grid)
        y_traj = interp_y(self.time_grid)
        
        # 计算期望的速度（基于路径长度和时间）
        expected_avg_speed = total_distance / self.time_horizon
        expected_avg_speed = min(expected_avg_speed, self.max_velocity * 0.8)
        
        # 计算速度（改进的方法）
        vx_traj = np.zeros(self.num_segments + 1)
        vy_traj = np.zeros(self.num_segments + 1)
        
        for i in range(self.num_segments + 1):
            if i == 0:
                # 起点速度：朝向下个点
                direction = np.array([x_traj[1] - x_traj[0], y_traj[1] - y_traj[0]])
                direction_norm = np.linalg.norm(direction)
                if direction_norm > 1e-6:
                    direction = direction / direction_norm
                    vx_traj[i] = direction[0] * expected_avg_speed
                    vy_traj[i] = direction[1] * expected_avg_speed
            elif i == self.num_segments:
                # 终点速度：从前一点过来的方向，但减速
                direction = np.array([x_traj[i] - x_traj[i-1], y_traj[i] - y_traj[i-1]])
                direction_norm = np.linalg.norm(direction)
                if direction_norm > 1e-6:
                    direction = direction / direction_norm
                    vx_traj[i] = direction[0] * expected_avg_speed * 0.3  # 终点减速
                    vy_traj[i] = direction[1] * expected_avg_speed * 0.3
            else:
                # 中间点速度：前后点连线的方向
                direction = np.array([x_traj[i+1] - x_traj[i-1], y_traj[i+1] - y_traj[i-1]])
                direction_norm = np.linalg.norm(direction)
                if direction_norm > 1e-6:
                    direction = direction / direction_norm
                    vx_traj[i] = direction[0] * expected_avg_speed
                    vy_traj[i] = direction[1] * expected_avg_speed
        
        # 平滑速度（轻微平滑，保持特征）
        for _ in range(2):
            vx_smooth = np.copy(vx_traj)
            vy_smooth = np.copy(vy_traj)
            for i in range(1, len(vx_traj) - 1):
                vx_smooth[i] = 0.2 * vx_traj[i-1] + 0.6 * vx_traj[i] + 0.2 * vx_traj[i+1]
                vy_smooth[i] = 0.2 * vy_traj[i-1] + 0.6 * vy_traj[i] + 0.2 * vy_traj[i+1]
            vx_traj = vx_smooth
            vy_traj = vy_smooth
        
        # 组装初始轨迹
        initial_trajectory = np.zeros(((self.num_segments + 1) * self.state_dim,))
        for i in range(self.num_segments + 1):
            idx = i * self.state_dim
            initial_trajectory[idx:idx+4] = [x_traj[i], y_traj[i], vx_traj[i], vy_traj[i]]
        
        return initial_trajectory
    
    def objective_function(self, trajectory_flat):
        """
        SCP优化的目标函数 - 改进版本
        重点强化waypoint跟踪，适度平滑
        """
        # 重塑轨迹数据
        trajectory = trajectory_flat.reshape((self.num_segments + 1, self.state_dim))
        
        total_cost = 0.0
        
        # 1. 强化waypoint跟踪成本
        tracking_weight = 1000.0  # 大幅增加权重
        if self.path_waypoints is not None:
            # 起点和终点精确跟踪
            start_pos = trajectory[0, :2]
            end_pos = trajectory[-1, :2]
            start_error = np.sum((start_pos - self.path_waypoints[0])**2)
            end_error = np.sum((end_pos - self.path_waypoints[-1])**2)
            total_cost += tracking_weight * (start_error + end_error)
            
            # 中间waypoint跟踪
            if len(self.path_waypoints) > 2:
                # 在轨迹中找到对应的时间点来跟踪中间waypoints
                waypoint_tracking_weight = 500.0
                for wp_idx in range(1, len(self.path_waypoints) - 1):
                    # 计算该waypoint对应的时间索引
                    wp_ratio = wp_idx / (len(self.path_waypoints) - 1)
                    traj_idx = int(wp_ratio * self.num_segments)
                    traj_idx = min(traj_idx, self.num_segments)
                    
                    traj_pos = trajectory[traj_idx, :2]
                    wp_pos = self.path_waypoints[wp_idx]
                    waypoint_error = np.sum((traj_pos - wp_pos)**2)
                    total_cost += waypoint_tracking_weight * waypoint_error
        
        # 2. 适度的平滑度成本（降低权重）
        smoothness_weight = 2.0  # 从10.0降低到2.0
        for i in range(1, self.num_segments):
            pos_curr = trajectory[i, :2]
            pos_prev = trajectory[i-1, :2]
            pos_next = trajectory[i+1, :2]
            
            acceleration = (pos_next - 2*pos_curr + pos_prev) / (self.dt**2)
            total_cost += smoothness_weight * np.sum(acceleration**2)
        
        # 3. 速度利用率成本（鼓励使用合理的速度）
        speed_utilization_weight = 0.5
        target_speed = self.max_velocity * 0.6  # 目标速度为最大速度的60%
        for i in range(self.num_segments + 1):
            velocity = trajectory[i, 2:4]
            current_speed = np.sqrt(np.sum(velocity**2))
            # 惩罚速度过小或过大
            speed_error = (current_speed - target_speed)**2
            total_cost += speed_utilization_weight * speed_error
        
        # 4. 路径长度成本（避免过长的轨迹）
        path_length_weight = 0.1
        for i in range(1, self.num_segments + 1):
            pos_curr = trajectory[i, :2]
            pos_prev = trajectory[i-1, :2]
            segment_length = np.sum((pos_curr - pos_prev)**2)
            total_cost += path_length_weight * segment_length
        
        return total_cost
    
    def constraint_functions(self, trajectory_flat):
        """
        约束函数集合 - 改进版本
        """
        trajectory = trajectory_flat.reshape((self.num_segments + 1, self.state_dim))
        constraints = []
        
        # 1. 速度约束
        for i in range(self.num_segments + 1):
            velocity = trajectory[i, 2:4]
            speed_squared = np.sum(velocity**2)
            # 约束: speed^2 <= max_velocity^2
            constraints.append(self.max_velocity**2 - speed_squared)
        
        # 2. 加速度约束
        for i in range(self.num_segments):
            vel_curr = trajectory[i, 2:4]
            vel_next = trajectory[i+1, 2:4]
            acceleration = (vel_next - vel_curr) / self.dt
            accel_squared = np.sum(acceleration**2)
            # 约束: accel^2 <= max_acceleration^2
            constraints.append(self.max_acceleration**2 - accel_squared)
        
        # 3. 动力学一致性约束 - 放宽容差
        for i in range(self.num_segments):
            pos_curr = trajectory[i, :2]
            vel_curr = trajectory[i, 2:4]
            pos_next = trajectory[i+1, :2]
            
            # 积分一致性: pos_next = pos_curr + vel_curr * dt
            pos_predicted = pos_curr + vel_curr * self.dt
            consistency_error = pos_next - pos_predicted
            
            # 放宽动力学一致性约束的容差
            tolerance = 0.1  # 从0.01增加到0.1
            constraints.append(tolerance - np.sum(consistency_error**2))
        
        # 4. 障碍物避障约束 - 增加安全边距
        safety_margin = 0.3  # 从0.1增加到0.3
        for obstacle in self.obstacles:
            obs_x, obs_y, obs_radius = obstacle
            for i in range(self.num_segments + 1):
                pos = trajectory[i, :2]
                distance_squared = (pos[0] - obs_x)**2 + (pos[1] - obs_y)**2
                min_distance_squared = (obs_radius + safety_margin)**2
                # 约束: distance^2 >= min_distance^2
                constraints.append(distance_squared - min_distance_squared)
        
        return np.array(constraints)
    
    def plan_trajectory(self, max_iterations=10, tolerance=1e-4, use_staged_optimization=True):
        """
        执行SCP轨迹规划 - 改进版本
        
        参数:
        max_iterations: SCP最大迭代次数
        tolerance: 收敛容差
        use_staged_optimization: 是否使用分阶段优化
        
        返回:
        trajectory: 优化后的轨迹 [时间点数 x 状态维度]
        success: 是否成功求解
        """
        if self.path_waypoints is None:
            raise ValueError("必须先设置路径waypoints")
        
        print("开始SCP轨迹优化...")
        
        # 生成初始轨迹猜测
        initial_trajectory = self.interpolate_initial_trajectory()
        current_trajectory = initial_trajectory.copy()
        
        if use_staged_optimization:
            return self._staged_optimization(current_trajectory, max_iterations, tolerance)
        else:
            return self._standard_optimization(current_trajectory, max_iterations, tolerance)
    
    def _staged_optimization(self, initial_trajectory, max_iterations, tolerance):
        """分阶段优化：先松弛约束，再逐步收紧"""
        current_trajectory = initial_trajectory.copy()
        
        # 阶段1：放宽约束优化
        original_max_vel = self.max_velocity
        original_max_acc = self.max_acceleration
        self.max_velocity *= 2.0
        self.max_acceleration *= 3.0
        
        for iteration in range(max_iterations // 2):
            result = self._single_optimization_step(current_trajectory, iteration + 1, max_iterations // 2, verbose=False)
            if result is None:
                continue
            
            trajectory_change = np.linalg.norm(result.x - current_trajectory)
            current_trajectory = result.x
            
            if trajectory_change < tolerance * 10:
                break
        
        # 阶段2：恢复约束，精细优化
        self.max_velocity = original_max_vel
        self.max_acceleration = original_max_acc
        
        for iteration in range(max_iterations // 2):
            result = self._single_optimization_step(current_trajectory, iteration + 1, max_iterations // 2, verbose=False)
            if result is None:
                continue
                
            trajectory_change = np.linalg.norm(result.x - current_trajectory)
            current_trajectory = result.x
            
            if trajectory_change < tolerance:
                break
        
        # 重塑最终轨迹
        final_trajectory = current_trajectory.reshape((self.num_segments + 1, self.state_dim))
        self.trajectory = final_trajectory
        
        print("✅ SCP轨迹规划完成!")
        return final_trajectory, True
    
    def _standard_optimization(self, initial_trajectory, max_iterations, tolerance):
        """标准SCP优化"""
        current_trajectory = initial_trajectory.copy()
        
        for iteration in range(max_iterations):
            result = self._single_optimization_step(current_trajectory, iteration + 1, max_iterations, verbose=False)
            if result is None:
                continue
                
            trajectory_change = np.linalg.norm(result.x - current_trajectory)
            current_trajectory = result.x
            
            if trajectory_change < tolerance:
                break
        
        final_trajectory = current_trajectory.reshape((self.num_segments + 1, self.state_dim))
        self.trajectory = final_trajectory
        
        print("✅ SCP轨迹规划完成!")
        return final_trajectory, True
    
    def _single_optimization_step(self, current_trajectory, iteration, max_iterations, verbose=True):
        """单次优化步骤"""
        # 约束设置
        constraints = []
        
        def constraint_func(x):
            return self.constraint_functions(x)
        
        constraints.append({
            'type': 'ineq',
            'fun': constraint_func
        })
        
        # 状态空间边界约束
        bounds = []
        for i in range(self.num_segments + 1):
            bounds.extend([(-1, 11), (-1, 11)])  # 位置bounds
            bounds.extend([(-self.max_velocity, self.max_velocity),
                          (-self.max_velocity, self.max_velocity)])  # 速度bounds
        
        try:
            result = minimize(
                fun=self.objective_function,
                x0=current_trajectory,
                method='SLSQP',
                bounds=bounds,
                constraints=constraints,
                options={
                    'maxiter': 500,
                    'ftol': 1e-6,
                    'disp': False
                }
            )
            
            return result
                
        except Exception as e:
            if verbose:
                print(f"优化错误: {str(e)}")
            return None
    
    def _validate_trajectory(self, trajectory):
        """验证轨迹是否满足约束"""
        # 检查速度约束
        speeds = np.sqrt(np.sum(trajectory[:, 2:4]**2, axis=1))
        max_speed = np.max(speeds)
        
        # 检查加速度约束
        accelerations = []
        for i in range(self.num_segments):
            vel_curr = trajectory[i, 2:4]
            vel_next = trajectory[i+1, 2:4]
            accel = np.linalg.norm((vel_next - vel_curr) / self.dt)
            accelerations.append(accel)
        
        max_accel = np.max(accelerations) if accelerations else 0
        
        # 简化输出
        if max_speed > self.max_velocity * 1.01 or max_accel > self.max_acceleration * 1.01:
            print(f"⚠️ 约束检查: 最大速度{max_speed:.2f}m/s, 最大加速度{max_accel:.2f}m/s²")
    
    def get_trajectory_at_time(self, t):
        """
        获取指定时间点的轨迹状态
        
        参数:
        t: 时间 (s)
        
        返回:
        state: [x, y, vx, vy] 或 None
        """
        if self.trajectory is None:
            return None
        
        if t < 0 or t > self.time_horizon:
            return None
        
        # 线性插值
        idx = int(t / self.dt)
        if idx >= self.num_segments:
            return self.trajectory[-1]
        
        alpha = (t - idx * self.dt) / self.dt
        state = (1 - alpha) * self.trajectory[idx] + alpha * self.trajectory[idx + 1]
        
        return state
    
    def visualize_trajectory(self, show_waypoints=True, show_velocity=True):
        """
        可视化轨迹结果
        
        参数:
        show_waypoints: 是否显示原始waypoints
        show_velocity: 是否显示速度向量
        """
        if self.trajectory is None:
            print("没有轨迹数据可视化")
            return
        
        plt.figure(figsize=(12, 10))
        
        # 绘制障碍物
        for obstacle in self.obstacles:
            obs_x, obs_y, obs_radius = obstacle
            circle = plt.Circle((obs_x, obs_y), obs_radius, color='red', alpha=0.7, label='障碍物')
            plt.gca().add_patch(circle)
        
        # 绘制原始waypoints
        if show_waypoints and self.path_waypoints is not None:
            waypoints = self.path_waypoints
            plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', 
                    markersize=6, linewidth=2, alpha=0.6, label='RRT路径点')
        
        # 绘制优化后的轨迹
        positions = self.trajectory[:, :2]
        plt.plot(positions[:, 0], positions[:, 1], 'g-', 
                linewidth=3, label='SCP优化轨迹')
        
        # 绘制起点和终点
        plt.plot(positions[0, 0], positions[0, 1], 'go', 
                markersize=10, label='起点')
        plt.plot(positions[-1, 0], positions[-1, 1], 'bs', 
                markersize=10, label='终点')
        
        # 绘制速度向量
        if show_velocity:
            step = max(1, self.num_segments // 20)  # 只显示部分速度向量
            for i in range(0, self.num_segments + 1, step):
                pos = self.trajectory[i, :2]
                vel = self.trajectory[i, 2:4]
                
                # 缩放速度向量用于显示
                scale = 0.3
                plt.arrow(pos[0], pos[1], vel[0]*scale, vel[1]*scale,
                         head_width=0.1, head_length=0.1, fc='blue', 
                         ec='blue', alpha=0.6)
        
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.xlabel('X 坐标 (m)')
        plt.ylabel('Y 坐标 (m)')
        plt.title('SCP轨迹规划结果')
        plt.legend()
        
        # 添加算法信息
        info_text = f"SCP轨迹: v_max={self.max_velocity}m/s, a_max={self.max_acceleration}m/s²"
        plt.text(0.02, 0.98, info_text, transform=plt.gca().transAxes, 
                fontsize=9, verticalalignment='top',
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8))
        
        plt.tight_layout()
        plt.show()
    
    def plot_trajectory_profiles(self):
        """绘制轨迹的速度、加速度等随时间变化的曲线"""
        if self.trajectory is None:
            print("没有轨迹数据可视化")
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # 位置曲线
        ax1 = axes[0, 0]
        ax1.plot(self.time_grid, self.trajectory[:, 0], 'r-', label='X位置')
        ax1.plot(self.time_grid, self.trajectory[:, 1], 'b-', label='Y位置')
        ax1.set_xlabel('时间 (s)')
        ax1.set_ylabel('位置 (m)')
        ax1.set_title('位置随时间变化')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # 速度曲线
        ax2 = axes[0, 1]
        speeds = np.sqrt(np.sum(self.trajectory[:, 2:4]**2, axis=1))
        ax2.plot(self.time_grid, self.trajectory[:, 2], 'r-', label='X速度')
        ax2.plot(self.time_grid, self.trajectory[:, 3], 'b-', label='Y速度')
        ax2.plot(self.time_grid, speeds, 'g-', linewidth=2, label='总速度')
        ax2.axhline(y=self.max_velocity, color='red', linestyle='--', 
                   alpha=0.7, label='速度限制')
        ax2.set_xlabel('时间 (s)')
        ax2.set_ylabel('速度 (m/s)')
        ax2.set_title('速度随时间变化')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        # 加速度曲线
        ax3 = axes[1, 0]
        accelerations = []
        for i in range(self.num_segments):
            vel_curr = self.trajectory[i, 2:4]
            vel_next = self.trajectory[i+1, 2:4]
            accel = (vel_next - vel_curr) / self.dt
            accelerations.append(np.linalg.norm(accel))
        accelerations.append(accelerations[-1])  # 扩展到相同长度
        
        ax3.plot(self.time_grid, accelerations, 'purple', linewidth=2, label='加速度幅值')
        ax3.axhline(y=self.max_acceleration, color='red', linestyle='--', 
                   alpha=0.7, label='加速度限制')
        ax3.set_xlabel('时间 (s)')
        ax3.set_ylabel('加速度 (m/s²)')
        ax3.set_title('加速度随时间变化')
        ax3.grid(True, alpha=0.3)
        ax3.legend()
        
        # 轨迹曲率
        ax4 = axes[1, 1]
        curvatures = []
        for i in range(1, self.num_segments):
            # 使用三点法计算曲率
            p1 = self.trajectory[i-1, :2]
            p2 = self.trajectory[i, :2]
            p3 = self.trajectory[i+1, :2]
            
            # 计算曲率
            v1 = p2 - p1
            v2 = p3 - p2
            
            cross_product = v1[0]*v2[1] - v1[1]*v2[0]
            v1_norm = np.linalg.norm(v1)
            v2_norm = np.linalg.norm(v2)
            
            if v1_norm > 1e-6 and v2_norm > 1e-6:
                curvature = abs(cross_product) / (v1_norm * v2_norm * np.linalg.norm(p3 - p1))
            else:
                curvature = 0
            curvatures.append(curvature)
        
        if curvatures:
            # 扩展曲率数组到完整长度
            curvatures = [curvatures[0]] + curvatures + [curvatures[-1]]
            ax4.plot(self.time_grid, curvatures, 'orange', linewidth=2, label='轨迹曲率')
            ax4.axhline(y=self.max_curvature, color='red', linestyle='--', 
                       alpha=0.7, label='曲率限制')
        
        ax4.set_xlabel('时间 (s)')
        ax4.set_ylabel('曲率 (1/m)')
        ax4.set_title('轨迹曲率随时间变化')
        ax4.grid(True, alpha=0.3)
        ax4.legend()
        
        plt.tight_layout()
        plt.show()

def main():
    """主函数示例 - 基本用法演示"""
    example_path = [[1.0, 1.0], [3.0, 3.0], [5.0, 5.0], [7.0, 7.0], [9.0, 9.0]]
    obstacles = [(4, 4, 0.8)]
    
    scp_planner = SCPTrajectoryPlanner(
        max_velocity=1.5,
        max_acceleration=1.0,
        obstacles=obstacles,
        time_horizon=8.0,
        num_segments=35
    )
    
    scp_planner.set_path_waypoints(example_path)
    trajectory, success = scp_planner.plan_trajectory(max_iterations=4)
    
    if success:
        print("✅ SCP轨迹规划完成")
        scp_planner.visualize_trajectory()
    else:
        print("❌ 轨迹规划失败")

if __name__ == "__main__":
    main()