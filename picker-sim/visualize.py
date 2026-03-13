#!/usr/bin/env python3
"""
Picker Robot V2 交互式 3D 可视化
================================

用 matplotlib 实时显示机械臂姿态，滑条控制 6 个关节。
支持鼠标拖拽旋转视角。

用法:
    cd picker-sim
    source .venv/bin/activate
    python visualize.py

控制:
    - 6 个滑条分别控制 J1~J6
    - 鼠标左键拖拽旋转 3D 视角
    - "Home" 按钮回零位
    - "Random" 按钮随机姿态
    - 底部显示末端 XYZ 坐标
"""

import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # macOS 兼容
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# 导入运动学
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kinematics import (
    forward_kinematics, fk_to_xyzrpy, mdh_transform,
    DH_PARAMS, JOINT_LIMITS, JOINT_SENSES, deg2rad
)


# ============================================================
# 颜色定义
# ============================================================
COLORS = {
    'base':  '#444444',
    'link1': '#2266CC',
    'link2': '#AAAAAA',
    'link3': '#2266CC',
    'link4': '#AAAAAA',
    'link5': '#DD7700',
    'link6': '#DD7700',
    'tool':  '#FF3333',
    'trail': '#FF666644',
}

LINK_NAMES = ['base', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6']


def compute_joint_positions(joint_angles_deg):
    """
    计算各关节坐标（用于画骨架）
    返回 7 个点：base, J1原点, J2原点, ..., J6原点（末端）
    """
    q = np.array(joint_angles_deg, dtype=float)
    positions = [np.array([0, 0, 0])]  # base

    T = np.eye(4)
    for i in range(6):
        alpha_i = DH_PARAMS[i, 0]
        a_i = DH_PARAMS[i, 1]
        d_i = DH_PARAMS[i, 2]
        theta_offset_i = DH_PARAMS[i, 3]
        theta_i = theta_offset_i + q[i] * JOINT_SENSES[i]
        Hi = mdh_transform(alpha_i, a_i, d_i, theta_i)
        T = T @ Hi
        positions.append(T[:3, 3].copy())

    return np.array(positions)


def draw_cylinder(ax, p1, p2, radius, color, alpha=0.6):
    """在两点间画圆柱体（用多边形近似）"""
    v = p2 - p1
    length = np.linalg.norm(v)
    if length < 1e-6:
        return

    v_norm = v / length

    # 找一个正交向量
    if abs(v_norm[0]) < 0.9:
        u = np.cross(v_norm, [1, 0, 0])
    else:
        u = np.cross(v_norm, [0, 1, 0])
    u = u / np.linalg.norm(u)
    w = np.cross(v_norm, u)

    n_sides = 8
    angles = np.linspace(0, 2 * np.pi, n_sides + 1)

    # 圆柱侧面
    for j in range(n_sides):
        a1, a2 = angles[j], angles[j + 1]
        c1 = radius * (np.cos(a1) * u + np.sin(a1) * w)
        c2 = radius * (np.cos(a2) * u + np.sin(a2) * w)
        verts = [
            p1 + c1, p1 + c2, p2 + c2, p2 + c1
        ]
        ax.add_collection3d(Poly3DCollection(
            [verts], alpha=alpha, facecolors=color, edgecolors=color, linewidths=0.3
        ))


class PickerVisualizer:
    def __init__(self):
        self.joint_angles = [0.0] * 6
        self.trail = []  # 末端轨迹
        self.max_trail = 200

        # 创建图形
        self.fig = plt.figure(figsize=(14, 9))
        self.fig.suptitle('Picker Robot V2 仿真', fontsize=14, fontweight='bold')

        # 3D 视图
        self.ax = self.fig.add_axes([0.05, 0.25, 0.65, 0.70], projection='3d')

        # 信息面板
        self.info_ax = self.fig.add_axes([0.72, 0.55, 0.26, 0.38])
        self.info_ax.axis('off')

        self._setup_sliders()
        self._setup_buttons()
        self._setup_3d()
        self.update(None)

    def _setup_3d(self):
        """初始化 3D 视图"""
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_xlim(-500, 500)
        self.ax.set_ylim(-500, 500)
        self.ax.set_zlim(-100, 700)
        self.ax.view_init(elev=25, azim=-60)

    def _setup_sliders(self):
        """创建 6 个关节滑条"""
        self.sliders = []
        slider_colors = ['#2266CC', '#2266CC', '#2266CC', '#DD7700', '#DD7700', '#DD7700']

        for i in range(6):
            lo, hi = JOINT_LIMITS[i + 1]
            ax_slider = self.fig.add_axes([0.08, 0.18 - i * 0.028, 0.55, 0.02])
            slider = Slider(
                ax_slider, f'J{i+1}',
                lo, hi,
                valinit=0.0,
                valstep=0.5,
                color=slider_colors[i],
            )
            slider.on_changed(self.update)
            self.sliders.append(slider)

    def _setup_buttons(self):
        """创建按钮"""
        ax_home = self.fig.add_axes([0.72, 0.42, 0.1, 0.04])
        self.btn_home = Button(ax_home, '🏠 Home', color='#DDDDDD')
        self.btn_home.on_clicked(self.go_home)

        ax_random = self.fig.add_axes([0.84, 0.42, 0.1, 0.04])
        self.btn_random = Button(ax_random, '🎲 Random', color='#DDDDDD')
        self.btn_random.on_clicked(self.go_random)

        ax_clear = self.fig.add_axes([0.72, 0.36, 0.1, 0.04])
        self.btn_clear = Button(ax_clear, '🧹 Clear Trail', color='#DDDDDD')
        self.btn_clear.on_clicked(self.clear_trail)

    def go_home(self, event):
        """回零位"""
        for s in self.sliders:
            s.set_val(0)

    def go_random(self, event):
        """随机姿态"""
        for i, s in enumerate(self.sliders):
            lo, hi = JOINT_LIMITS[i + 1]
            s.set_val(np.random.uniform(lo * 0.5, hi * 0.5))

    def clear_trail(self, event):
        """清除轨迹"""
        self.trail = []
        self.update(None)

    def update(self, val):
        """更新可视化"""
        # 读取关节角度
        for i in range(6):
            self.joint_angles[i] = self.sliders[i].val

        # 计算关节位置
        positions = compute_joint_positions(self.joint_angles)

        # 计算末端位姿
        xyzrpy = fk_to_xyzrpy(self.joint_angles)

        # 添加轨迹
        self.trail.append(positions[-1].copy())
        if len(self.trail) > self.max_trail:
            self.trail = self.trail[-self.max_trail:]

        # 清除并重绘
        self.ax.cla()
        self._setup_3d()

        # 画地面网格
        grid_range = np.arange(-400, 401, 100)
        for g in grid_range:
            self.ax.plot([g, g], [-400, 400], [0, 0], color='#DDDDDD', linewidth=0.3)
            self.ax.plot([-400, 400], [g, g], [0, 0], color='#DDDDDD', linewidth=0.3)

        # 画机械臂
        link_radii = [30, 25, 20, 20, 15, 12, 10]
        link_colors = [COLORS[n] for n in LINK_NAMES]

        for i in range(len(positions) - 1):
            p1, p2 = positions[i], positions[i + 1]
            draw_cylinder(self.ax, p1, p2, link_radii[i], link_colors[i])

        # 画关节点
        for i, pos in enumerate(positions):
            color = '#FF3333' if i == len(positions) - 1 else '#333333'
            size = 80 if i == len(positions) - 1 else 40
            self.ax.scatter(*pos, color=color, s=size, zorder=5, depthshade=False)

        # 画骨架线
        self.ax.plot(
            positions[:, 0], positions[:, 1], positions[:, 2],
            'k-', linewidth=1.5, alpha=0.5
        )

        # 画末端坐标系
        T = forward_kinematics(self.joint_angles)
        origin = T[:3, 3]
        axis_len = 40
        for j, (color, label) in enumerate(zip(['r', 'g', 'b'], ['x', 'y', 'z'])):
            end = origin + axis_len * T[:3, j]
            self.ax.plot(
                [origin[0], end[0]], [origin[1], end[1]], [origin[2], end[2]],
                color=color, linewidth=2, alpha=0.8
            )

        # 画轨迹
        if len(self.trail) > 1:
            trail_arr = np.array(self.trail)
            self.ax.plot(
                trail_arr[:, 0], trail_arr[:, 1], trail_arr[:, 2],
                color='#FF6666', linewidth=1, alpha=0.4
            )

        # 更新信息面板
        self.info_ax.cla()
        self.info_ax.axis('off')

        info_lines = [
            ('─── 末端位姿 ───', 'black', 11, 'bold'),
            (f'X: {xyzrpy[0]:8.1f} mm', '#CC0000', 10, 'normal'),
            (f'Y: {xyzrpy[1]:8.1f} mm', '#00AA00', 10, 'normal'),
            (f'Z: {xyzrpy[2]:8.1f} mm', '#0000CC', 10, 'normal'),
            (f'Rx: {xyzrpy[3]:7.1f}°', '#666666', 10, 'normal'),
            (f'Ry: {xyzrpy[4]:7.1f}°', '#666666', 10, 'normal'),
            (f'Rz: {xyzrpy[5]:7.1f}°', '#666666', 10, 'normal'),
            ('', 'black', 6, 'normal'),
            ('─── 关节角度 ───', 'black', 11, 'bold'),
        ]
        for i in range(6):
            lo, hi = JOINT_LIMITS[i + 1]
            pct = (self.joint_angles[i] - lo) / (hi - lo) * 100
            info_lines.append(
                (f'J{i+1}: {self.joint_angles[i]:7.1f}° ({pct:.0f}%)', '#333333', 9, 'normal')
            )

        for j, (text, color, size, weight) in enumerate(info_lines):
            self.info_ax.text(
                0, 1.0 - j * 0.065, text,
                transform=self.info_ax.transAxes,
                fontsize=size, color=color, fontweight=weight,
                fontfamily='monospace', verticalalignment='top'
            )

        self.fig.canvas.draw_idle()

    def show(self):
        """显示窗口"""
        plt.show()


if __name__ == '__main__':
    print("🤖 Picker V2 可视化启动中...")
    print("   操作: 拖动滑条控制关节 | 鼠标旋转视角")
    print("   按钮: Home=回零位 | Random=随机 | Clear Trail=清轨迹")
    viz = PickerVisualizer()
    viz.show()
