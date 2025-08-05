# 项目文件结构

## 核心算法模块
```
├── rrt.py                    # RRT路径规划算法
├── rrt_star.py              # RRT*路径规划算法  
└── scp_trajectory.py        # SCP轨迹优化算法
```

## 运行脚本
```
├── run_rrt.py              # RRT单独运行
├── run_rrt_star.py         # RRT*单独运行
├── run_scp_trajectory.py   # 集成SCP轨迹规划
└── compare_rrt_rrt_star.py # RRT vs RRT*对比
```

## 文档说明
```
├── README.md               # 项目总体说明
├── SCP_README.md          # SCP模块详细文档
├── USAGE.md               # 快速使用指南
└── RRT_vs_RRT_Star.md     # RRT vs RRT*对比分析
```

## 配置文件
```
├── requirements.txt        # 项目依赖
└── .gitignore             # Git忽略文件
```

## 使用方式

### 1. 基础路径规划
```bash
python run_rrt.py          # RRT算法
python run_rrt_star.py     # RRT*算法
```

### 2. 集成轨迹规划 (推荐)
```bash
python run_scp_trajectory.py
```

### 3. 算法对比
```bash
python compare_rrt_rrt_star.py
```

## 功能特性

| 功能 | RRT | RRT* | RRT+SCP | RRT*+SCP |
|------|-----|------|---------|----------|
| 路径规划 | ✅ | ✅ | ✅ | ✅ |
| 路径优化 | ❌ | ✅ | ❌ | ✅ |
| 动力学约束 | ❌ | ❌ | ✅ | ✅ |
| 平滑轨迹 | ❌ | ❌ | ✅ | ✅ |
| 时间参数化 | ❌ | ❌ | ✅ | ✅ |

## 推荐用法

- **快速原型**: 使用 `run_rrt.py`
- **高质量路径**: 使用 `run_rrt_star.py`  
- **实际部署**: 使用 `run_scp_trajectory.py` (RRT+SCP 或 RRT*+SCP)