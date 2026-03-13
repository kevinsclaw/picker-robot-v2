"""
视觉模块测试脚本

测试内容:
  1. 相机采集预览
  2. 颜色检测 + 实时标注
  3. HSV 调参工具
  4. 手眼标定
"""

import cv2
import numpy as np
import logging
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision.camera import Camera
from vision.detector import ColorDetector, ColorTuner
from vision.calibration import HandEyeCalibration, InteractiveCalibrator

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")


def test_camera_preview():
    """测试相机预览"""
    print("选择相机:")
    print("  0: MacBook 内置摄像头")
    print("  1: USB 摄像头")
    print("  g: Gemini 2 深度相机")
    choice = input("输入: ").strip()

    if choice == "g":
        cam = Camera(device="gemini2", width=640, height=480)
    else:
        cam = Camera(device=int(choice), width=640, height=480)

    with cam:
        print("按 'q' 退出, 's' 截图")
        while True:
            color, depth = cam.read()
            if color is None:
                continue

            cv2.imshow("Color", color)

            if depth is not None:
                # 深度可视化
                depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
                depth_vis = depth_vis.astype(np.uint8)
                depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                cv2.imshow("Depth", depth_vis)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                cv2.imwrite("capture_color.png", color)
                if depth is not None:
                    np.save("capture_depth.npy", depth)
                print("截图已保存")

    cv2.destroyAllWindows()


def test_color_detection():
    """测试颜色检测"""
    cam = Camera(device=0, width=640, height=480)
    detector = ColorDetector(min_area=500, use_presets=True)

    with cam:
        print("实时颜色检测 — 按 'q' 退出")
        print(f"检测颜色: {[c.name for c in detector._colors]}")

        while True:
            color, depth = cam.read()
            if color is None:
                continue

            objects = detector.detect(color, depth)

            # 绘制检测结果
            vis = detector.draw_detections(color, objects)

            # 显示统计
            info = f"Detected: {len(objects)}"
            for obj in objects:
                info += f" | {obj}"
            cv2.putText(vis, info[:100], (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.imshow("Detection", vis)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break

    cv2.destroyAllWindows()


def test_color_tuner():
    """HSV 颜色调参"""
    cam = Camera(device=0, width=640, height=480)
    tuner = ColorTuner()

    with cam:
        lower, upper = tuner.run(camera=cam)
        print(f"\n最终值:")
        print(f'  lower=({lower[0]}, {lower[1]}, {lower[2]})')
        print(f'  upper=({upper[0]}, {upper[1]}, {upper[2]})')
        print(f'\n代码:')
        print(f'  detector.add_color("my_color", {lower}, {upper})')


def test_calibration_manual():
    """手动标定测试 (无机械臂也能跑)"""
    calib = HandEyeCalibration()

    print("手动输入标定点 (至少 4 个)")
    print("格式: 像素u 像素v 世界x 世界y 世界z")
    print("输入 'done' 完成, 'demo' 用演示数据\n")

    while True:
        line = input("标定点: ").strip()
        if line == "done":
            break
        if line == "demo":
            # 演示数据: 假设相机在 640×480 分辨率下俯拍
            # 工作区 400×300mm 映射到图像中央区域
            demo_points = [
                ((120, 80),  (100, 50, 0)),
                ((520, 80),  (300, 50, 0)),
                ((520, 400), (300, 250, 0)),
                ((120, 400), (100, 250, 0)),
                ((320, 240), (200, 150, 0)),  # 中心验证点
            ]
            for pixel, world in demo_points:
                calib.add_point(pixel=pixel, world=world)
            break

        try:
            parts = line.split()
            u, v = float(parts[0]), float(parts[1])
            x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
            calib.add_point(pixel=(u, v), world=(x, y, z))
        except (ValueError, IndexError):
            print("格式错误，请重新输入")

    if calib.num_points >= 3:
        error = calib.calibrate()
        print(f"\n✅ 标定完成! 误差: {error:.2f} mm")

        # 测试转换
        print("\n测试坐标转换 (输入 'q' 退出):")
        while True:
            line = input("像素 u v: ").strip()
            if line == 'q':
                break
            try:
                u, v = map(float, line.split())
                wx, wy, wz = calib.pixel_to_world(u, v)
                print(f"  → 世界坐标: ({wx:.1f}, {wy:.1f}, {wz:.1f}) mm")
            except Exception as e:
                print(f"  错误: {e}")

        # 保存
        save = input("\n保存标定? (y/N): ").strip()
        if save.lower() == 'y':
            calib.save("calibration.json")
            print("已保存到 calibration.json")


def test_interactive_calibration():
    """交互式标定 (需要相机)"""
    cam = Camera(device=0, width=640, height=480)

    with cam:
        calibrator = InteractiveCalibrator(camera=cam)
        calib = calibrator.run(num_points=4)

        if calib.is_calibrated:
            calib.save("calibration.json")
            print("标定结果已保存到 calibration.json")


if __name__ == "__main__":
    print("=" * 50)
    print("  Picker Demo — 视觉模块测试")
    print("=" * 50)
    print()
    print("选择测试:")
    print("  1. 相机预览")
    print("  2. 颜色检测 (实时)")
    print("  3. HSV 调参工具")
    print("  4. 手动标定测试")
    print("  5. 交互式标定 (需要相机)")
    print()

    choice = input("输入编号 (1-5): ").strip()

    tests = {
        "1": test_camera_preview,
        "2": test_color_detection,
        "3": test_color_tuner,
        "4": test_calibration_manual,
        "5": test_interactive_calibration,
    }

    if choice in tests:
        tests[choice]()
    else:
        print("无效选择")
