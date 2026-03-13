"""
Picker Robot V2 运动学库
=======================

基于固件 AR4 v5.1 的 DH 参数实现正运动学（FK）和逆运动学（IK）。
使用 Modified DH (Craig) 约定。

单位：毫米 (mm)，角度 (degrees)
"""

import numpy as np
from numpy import sin, cos, arctan2, sqrt, pi

# ============================================================
# DH 参数定义（Modified DH / Craig Convention）
# 固件原始格式: {theta_offset(deg), alpha(deg), d(mm), a(mm)}
# 这里重排为: [alpha(deg), a(mm), d(mm), theta_offset(deg)]
# ============================================================

DH_PARAMS = np.array([
    [0,    0,      169.77,  0],      # Joint 1
    [-90,  64.2,   0,       -90],    # Joint 2
    [0,    305,    0,       0],      # Joint 3
    [-90,  0,      222.63,  0],      # Joint 4
    [90,   0,      0,       0],      # Joint 5
    [-90,  0,      41,      180],    # Joint 6
])

# 关节限位（度）
JOINT_LIMITS = {
    1: (-163, 163),
    2: (-60,  90),
    3: (-90,  85),
    4: (-173, 173),
    5: (-108, 108),
    6: (-180, 180),
}

# 各关节方向（+1 表示正方向一致）
JOINT_SENSES = [1, 1, 1, 1, 1, 1]


def deg2rad(deg):
    return deg * pi / 180.0


def rad2deg(rad):
    return rad * 180.0 / pi


def mdh_transform(alpha_deg, a, d, theta_deg):
    """
    Modified DH 变换矩阵 (Craig Convention)
    T = Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
    """
    alpha = deg2rad(alpha_deg)
    theta = deg2rad(theta_deg)

    ca, sa = cos(alpha), sin(alpha)
    ct, st = cos(theta), sin(theta)

    return np.array([
        [ct,      -st,       0,     a],
        [ca * st,  ca * ct, -sa,   -d * sa],
        [sa * st,  sa * ct,  ca,    d * ca],
        [0,        0,        0,     1]
    ])


def forward_kinematics(joint_angles_deg):
    """
    正运动学（FK）：关节角度 → 末端位姿 4x4 矩阵

    参数:
        joint_angles_deg: 6个关节角度 (度)

    返回:
        4x4 齐次变换矩阵
    """
    q = np.array(joint_angles_deg, dtype=float)
    T = np.eye(4)

    for i in range(6):
        alpha_i = DH_PARAMS[i, 0]
        a_i = DH_PARAMS[i, 1]
        d_i = DH_PARAMS[i, 2]
        theta_offset_i = DH_PARAMS[i, 3]
        theta_i = theta_offset_i + q[i] * JOINT_SENSES[i]
        Hi = mdh_transform(alpha_i, a_i, d_i, theta_i)
        T = T @ Hi

    return T


def fk_to_xyzrpy(joint_angles_deg):
    """正运动学：关节角度 → [x, y, z, rx, ry, rz] (ZYX 欧拉角，度)"""
    T = forward_kinematics(joint_angles_deg)
    return pose_to_xyzrpy(T)


def pose_to_xyzrpy(T):
    """从 4x4 变换矩阵提取 [x, y, z, rx, ry, rz]，ZYX 欧拉角"""
    x, y, z = T[0, 3], T[1, 3], T[2, 3]

    sy = np.clip(T[0, 2], -1.0, 1.0)
    if abs(sy) < 0.99999:
        ry = np.arcsin(sy)
        rx = arctan2(-T[1, 2], T[2, 2])
        rz = arctan2(-T[0, 1], T[0, 0])
    else:
        ry = pi / 2 * np.sign(sy)
        rx = arctan2(T[2, 1], T[1, 1])
        rz = 0.0

    return np.array([x, y, z, rad2deg(rx), rad2deg(ry), rad2deg(rz)])


def xyzrpy_to_pose(xyzrpy):
    """[x, y, z, rx, ry, rz] → 4x4 变换矩阵，ZYX 欧拉角"""
    x, y, z = xyzrpy[0], xyzrpy[1], xyzrpy[2]
    rx = deg2rad(xyzrpy[3])
    ry = deg2rad(xyzrpy[4])
    rz = deg2rad(xyzrpy[5])

    srx, crx = sin(rx), cos(rx)
    sry, cry = sin(ry), cos(ry)
    srz, crz = sin(rz), cos(rz)

    T = np.array([
        [cry * crz,  -cry * srz,          sry,    x],
        [crx * srz + srx * sry * crz,
         crx * crz - srx * sry * srz,    -cry * srx, y],
        [srx * srz - crx * sry * crz,
         srx * crz + crx * sry * srz,     crx * cry, z],
        [0, 0, 0, 1]
    ])
    return T


def check_joint_limits(joint_angles_deg):
    """检查关节限位，返回 (valid, violations)"""
    violations = []
    for i in range(6):
        lo, hi = JOINT_LIMITS[i + 1]
        if joint_angles_deg[i] < lo - 0.01 or joint_angles_deg[i] > hi + 0.01:
            violations.append((i + 1, joint_angles_deg[i], lo, hi))
    return len(violations) == 0, violations


# ============================================================
# 数值逆运动学（基于 Jacobian + Damped Least Squares）
# ============================================================

def _jacobian(joint_angles_deg, delta=0.01):
    """数值雅可比矩阵 (6x6)"""
    J = np.zeros((6, 6))
    base = fk_to_xyzrpy(joint_angles_deg)

    for i in range(6):
        q_plus = np.array(joint_angles_deg, dtype=float)
        q_plus[i] += delta
        fk_plus = fk_to_xyzrpy(q_plus)
        diff = fk_plus - base

        # 角度差归一化到 [-180, 180]
        for j in range(3, 6):
            while diff[j] > 180:
                diff[j] -= 360
            while diff[j] < -180:
                diff[j] += 360

        J[:, i] = diff / delta

    return J


def inverse_kinematics(target_xyzrpy, joint_estimate=None, max_iter=200,
                       tol_pos=0.05, tol_ang=0.1, damping=0.5):
    """
    数值逆运动学（Damped Least Squares / Levenberg-Marquardt）

    参数:
        target_xyzrpy: [x, y, z, rx, ry, rz] (mm, 度)
        joint_estimate: 初始关节估计值 (度)，默认 [0]*6
        max_iter: 最大迭代次数
        tol_pos: 位置收敛容差 (mm)
        tol_ang: 姿态收敛容差 (度)
        damping: 阻尼系数

    返回:
        关节角度 (度) 的 numpy 数组，或 None（未收敛）
    """
    if joint_estimate is None:
        joint_estimate = [0.0] * 6

    q = np.array(joint_estimate, dtype=float)
    target = np.array(target_xyzrpy, dtype=float)

    for iteration in range(max_iter):
        current = fk_to_xyzrpy(q)
        error = target - current

        # 角度差归一化
        for j in range(3, 6):
            while error[j] > 180:
                error[j] -= 360
            while error[j] < -180:
                error[j] += 360

        pos_err = np.linalg.norm(error[:3])
        ang_err = np.linalg.norm(error[3:6])

        if pos_err < tol_pos and ang_err < tol_ang:
            # 检查关节限位
            valid, _ = check_joint_limits(q)
            if valid:
                return q
            else:
                # 夹紧到限位
                for i in range(6):
                    lo, hi = JOINT_LIMITS[i + 1]
                    q[i] = np.clip(q[i], lo, hi)
                return q

        J = _jacobian(q)
        # Damped Least Squares: dq = J^T (J J^T + λ²I)^-1 e
        JJT = J @ J.T
        dq = J.T @ np.linalg.solve(JJT + damping**2 * np.eye(6), error)

        # 限制步长
        max_step = 5.0  # 度
        step_norm = np.max(np.abs(dq))
        if step_norm > max_step:
            dq *= max_step / step_norm

        q += dq

        # 软限位约束
        for i in range(6):
            lo, hi = JOINT_LIMITS[i + 1]
            q[i] = np.clip(q[i], lo - 5, hi + 5)

    # 未收敛，检查是否接近
    current = fk_to_xyzrpy(q)
    error = target - current
    for j in range(3, 6):
        while error[j] > 180:
            error[j] -= 360
        while error[j] < -180:
            error[j] += 360

    pos_err = np.linalg.norm(error[:3])
    if pos_err < 1.0:  # 放宽到 1mm
        valid, _ = check_joint_limits(q)
        if valid:
            return q

    return None


def inverse_kinematics_best(target_xyzrpy, joint_estimate=None):
    """
    尝试多组初始值，选择最优 IK 解

    返回:
        最优关节角度 (度) 或 None
    """
    if joint_estimate is None:
        joint_estimate = [0.0] * 6

    solutions = []

    # 从给定估计开始
    sol = inverse_kinematics(target_xyzrpy, joint_estimate)
    if sol is not None:
        solutions.append(sol)

    # 尝试不同初始值
    seeds = [
        [0, 0, 0, 0, 0, 0],
        [0, -30, 45, 0, -45, 0],
        [0, 30, -30, 0, 45, 0],
        [90, 0, 0, 0, 0, 0],
        [-90, 0, 0, 0, 0, 0],
    ]
    for seed in seeds:
        sol = inverse_kinematics(target_xyzrpy, seed)
        if sol is not None:
            is_dup = False
            for prev in solutions:
                if np.allclose(sol, prev, atol=1.0):
                    is_dup = True
                    break
            if not is_dup:
                solutions.append(sol)

    if not solutions:
        return None

    best = min(solutions,
               key=lambda s: np.sum((np.array(s) - np.array(joint_estimate))**2))
    return best


# ============================================================
# 便捷函数
# ============================================================

def fk(joint_angles_deg):
    """FK 快捷方式：返回 [x, y, z, rx, ry, rz]"""
    return fk_to_xyzrpy(joint_angles_deg)


def ik(target_xyzrpy, joint_estimate=None):
    """IK 快捷方式：返回最优关节角度"""
    return inverse_kinematics_best(target_xyzrpy, joint_estimate)


if __name__ == "__main__":
    print("=== Picker V2 运动学测试 ===\n")

    # 零位 FK
    zero_pos = [0, 0, 0, 0, 0, 0]
    result = fk(zero_pos)
    print(f"零位 FK: x={result[0]:.2f}, y={result[1]:.2f}, z={result[2]:.2f} mm")
    print(f"         rx={result[3]:.2f}, ry={result[4]:.2f}, rz={result[5]:.2f} deg")

    # FK → IK → FK 闭环测试
    test_configs = [
        [0, 0, 0, 0, 0, 0],
        [30, -20, 30, 0, -30, 0],
        [-45, 15, -15, 45, 20, -30],
    ]

    print("\n--- FK → IK → FK 闭环测试 ---")
    for q_orig in test_configs:
        target = fk(q_orig)
        q_ik = ik(target, q_orig)
        if q_ik is not None:
            fk_check = fk(q_ik)
            pos_err = np.linalg.norm(target[:3] - fk_check[:3])
            print(f"q={q_orig} → FK={np.round(target[:3],1)} → IK → FK误差={pos_err:.4f} mm ✓" if pos_err < 0.1 else f"q={q_orig} → 误差={pos_err:.4f} mm ✗")
        else:
            print(f"q={q_orig} → IK 无解！ ✗")
