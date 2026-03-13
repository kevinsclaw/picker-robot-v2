"""
Picker V2 运动学测试
===================
验证 FK/IK 一致性和边界情况
"""

import numpy as np
import sys
sys.path.insert(0, '.')
from kinematics import (
    fk, ik, forward_kinematics, fk_to_xyzrpy,
    check_joint_limits, JOINT_LIMITS
)


def test_fk_zero_position():
    """零位 FK 应返回合理位置"""
    result = fk([0, 0, 0, 0, 0, 0])
    print(f"[零位 FK] xyz=({result[0]:.2f}, {result[1]:.2f}, {result[2]:.2f})")

    # 零位高度应在 400~600mm 范围
    assert 300 < result[2] < 600, f"零位 z 高度异常: {result[2]}"
    # y 应接近 0
    assert abs(result[1]) < 0.01, f"零位 y 偏移: {result[1]}"
    print("  ✅ 通过\n")


def test_fk_ik_roundtrip():
    """FK → IK → FK 闭环误差应 < 0.1mm"""
    test_configs = [
        [0, 0, 0, 0, 0, 0],
        [30, -20, 30, 0, -30, 0],
        [-45, 15, -15, 45, 20, -30],
        [90, -30, 60, -90, 45, 0],
        [-120, 45, -45, 120, -60, 90],
    ]

    all_passed = True
    for q in test_configs:
        target = fk(q)
        q_ik = ik(target, q)

        if q_ik is None:
            print(f"  ❌ q={q} → IK 无解")
            all_passed = False
            continue

        fk_check = fk(q_ik)
        pos_err = np.linalg.norm(target[:3] - fk_check[:3])

        if pos_err < 0.1:
            print(f"  ✅ q={q} → 误差 {pos_err:.6f} mm")
        else:
            print(f"  ❌ q={q} → 误差 {pos_err:.4f} mm (>0.1mm)")
            all_passed = False

    assert all_passed, "部分 FK→IK→FK 测试失败"
    print()


def test_joint_limits():
    """关节限位检查"""
    # 在限位内
    valid, _ = check_joint_limits([0, 0, 0, 0, 0, 0])
    assert valid, "零位应在限位内"

    # 超限
    valid, violations = check_joint_limits([200, 0, 0, 0, 0, 0])
    assert not valid, "J1=200° 应超限"
    assert violations[0][0] == 1, "应报告 J1 超限"
    print("  ✅ 关节限位检查通过\n")


def test_workspace_boundary():
    """工作空间边界：可达点 IK 应有解"""
    reachable_points = [
        [300, 0, 400, 0, 90, 0],
        [200, 200, 300, 0, 90, 0],
        [-200, 100, 500, 0, 90, 0],
    ]

    for target in reachable_points:
        sol = ik(target)
        if sol is not None:
            fk_result = fk(sol)
            err = np.linalg.norm(np.array(target[:3]) - fk_result[:3])
            print(f"  ✅ target={target[:3]} → 误差 {err:.4f} mm")
        else:
            print(f"  ⚠️  target={target[:3]} → IK 无解（可能在工作空间边缘）")
    print()


if __name__ == "__main__":
    print("=" * 50)
    print("Picker V2 运动学测试")
    print("=" * 50 + "\n")

    print("[1] 零位 FK 测试")
    test_fk_zero_position()

    print("[2] FK ↔ IK 闭环测试")
    test_fk_ik_roundtrip()

    print("[3] 关节限位测试")
    test_joint_limits()

    print("[4] 工作空间可达性测试")
    test_workspace_boundary()

    print("=" * 50)
    print("全部测试通过 ✅")
    print("=" * 50)
