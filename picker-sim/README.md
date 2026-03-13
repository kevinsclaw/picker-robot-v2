# Picker V2 仿真环境

基于 Picker Robot V2（AR4 v5.1）的 ROS 2 + Gazebo 仿真。

## 文件结构

```
picker-sim/
├── kinematics.py          # Python 正/逆运动学库
├── run_sim.sh             # 一键启动脚本
├── docker/
│   └── Dockerfile         # ROS 2 Humble + Gazebo 环境
├── urdf/
│   └── picker_v2.urdf     # 机器人模型
├── launch/
│   ├── display.launch.py  # RViz2 可视化 (滑条控制)
│   └── gazebo.launch.py   # Gazebo 物理仿真
├── worlds/
│   └── pick_place.sdf     # 仿真场景 (桌面+彩色方块)
├── meshes/                # STL 模型文件
└── config/                # RViz 配置
```

## 快速开始

### 前提条件
- macOS + Docker Desktop（已安装）
- XQuartz（用于 GUI 显示）：`brew install --cask xquartz`

### 1. 构建镜像
```bash
cd picker-sim
./run_sim.sh bash   # 首次运行会自动构建
```

### 2. 运行仿真
```bash
# 方式 A: RViz2 可视化 — 滑条交互控制每个关节
./run_sim.sh display

# 方式 B: Gazebo 物理仿真 — 包含桌面和彩色方块
./run_sim.sh gazebo

# 方式 C: 交互模式 — 进入容器手动操作
./run_sim.sh bash
```

### 3. 容器内可用命令
```bash
# 查看关节状态
ros2 topic echo /joint_states

# 发送关节指令
ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray \
  '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'

# 直接跑 Gazebo
gz sim -r /ros2_ws/src/picker_sim/worlds/pick_place.sdf
```

## 运动学库

```python
from kinematics import fk, ik

# 正运动学：关节角度 → 末端位姿
pose = fk([0, 0, 0, 0, 0, 0])
# → [327.83, 0, 474.77, 0, 90, 0]  (x,y,z mm; rx,ry,rz deg)

# 逆运动学：末端位姿 → 关节角度
joints = ik([300, 100, 400, 0, 90, 0])
```

## DH 参数

| Joint | α (°) | a (mm) | d (mm)  | θ offset (°) | 限位 (°)     |
|-------|--------|--------|---------|---------------|-------------|
| 1     | 0      | 0      | 169.77  | 0             | ±163        |
| 2     | -90    | 64.2   | 0       | -90           | -60 ~ +90   |
| 3     | 0      | 305    | 0       | 0             | -90 ~ +85   |
| 4     | -90    | 0      | 222.63  | 0             | ±173        |
| 5     | 90     | 0      | 0       | 0             | ±108        |
| 6     | -90    | 0      | 41      | 180           | ±180        |

## macOS GUI 显示

Docker 容器的 GUI 需要 X11 转发：

1. 安装 XQuartz: `brew install --cask xquartz`
2. 注销并重新登录
3. 打开 XQuartz → 偏好设置 → 安全 → 勾选「允许从网络客户端连接」
4. 终端执行: `xhost +localhost`
5. 然后运行 `./run_sim.sh`
