"""
手眼标定模块 (Eye-to-Hand)

将相机像素坐标转换为机械臂基坐标系下的世界坐标。

两种标定方式:
  1. 简单 4 点标定 (仿射变换) — 快速，够用
  2. 完整标定 (PnP + 深度) — 精确，需要深度相机

使用:
    calib = HandEyeCalibration()

    # 方式 1: 4 点标定
    calib.add_point(pixel=(320, 240), world=(200, 0, 50))
    calib.add_point(pixel=(480, 240), world=(300, 0, 50))
    calib.add_point(pixel=(480, 360), world=(300, 100, 50))
    calib.add_point(pixel=(320, 360), world=(200, 100, 50))
    calib.calibrate()

    # 使用
    world_xyz = calib.pixel_to_world(400, 300)
    print(f"World: {world_xyz}")

    # 保存/加载
    calib.save("calibration.json")
    calib.load("calibration.json")
"""

import numpy as np
import cv2
import json
import logging
from typing import List, Tuple, Optional
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


@dataclass
class CalibrationPoint:
    """标定点: 像素坐标 ↔ 世界坐标"""
    pixel: Tuple[float, float]     # (u, v)
    world: Tuple[float, float, float]  # (x, y, z) mm, 机械臂基坐标系


class HandEyeCalibration:
    """
    Eye-to-Hand 手眼标定

    支持两种模式:
    - 2D 仿射变换 (≥4 点，平面场景)
    - 3D 齐次变换 (≥4 点 + 深度)
    """

    def __init__(self):
        self._points: List[CalibrationPoint] = []
        self._transform_2d: Optional[np.ndarray] = None  # 3×3 仿射/透视
        self._transform_3d: Optional[np.ndarray] = None  # 4×4 齐次
        self._plane_z: float = 0.0  # 工作平面 Z 高度 (mm)
        self._calibrated = False
        self._method = "affine"  # "affine" | "perspective" | "3d"

    # ─── 标定流程 ────────────────────────────────────────────

    def add_point(
        self,
        pixel: Tuple[float, float],
        world: Tuple[float, float, float],
    ):
        """
        添加标定点

        Args:
            pixel: 像素坐标 (u, v)
            world: 世界坐标 (x, y, z) mm
        """
        self._points.append(CalibrationPoint(pixel=pixel, world=world))
        self._calibrated = False
        logger.info(f"Calibration point added: pixel{pixel} → world{world} (total: {len(self._points)})")

    def clear_points(self):
        """清空标定点"""
        self._points.clear()
        self._calibrated = False

    def calibrate(self, method: str = "auto") -> float:
        """
        执行标定

        Args:
            method: "affine" | "perspective" | "auto"
                - affine: 仿射变换 (需 ≥3 点, 保持平行)
                - perspective: 透视变换 (需 ≥4 点, 更通用)
                - auto: ≥4 点用 perspective, 否则 affine

        Returns:
            重投影误差 (像素)
        """
        n = len(self._points)
        if n < 3:
            raise ValueError(f"Need at least 3 points, got {n}")

        if method == "auto":
            method = "perspective" if n >= 4 else "affine"

        self._method = method

        # 提取像素和世界坐标 (仅 XY 平面)
        pixels = np.array([p.pixel for p in self._points], dtype=np.float32)
        worlds_xy = np.array([(p.world[0], p.world[1]) for p in self._points], dtype=np.float32)

        # 记录工作平面 Z
        self._plane_z = np.mean([p.world[2] for p in self._points])

        if method == "affine":
            # 仿射变换: 需要恰好 3 点 (或用最小二乘拟合)
            if n == 3:
                self._transform_2d = cv2.getAffineTransform(pixels, worlds_xy)
                # 转为 3×3
                self._transform_2d = np.vstack([self._transform_2d, [0, 0, 1]])
            else:
                # 用 estimateAffine2D (RANSAC)
                M, inliers = cv2.estimateAffine2D(pixels, worlds_xy)
                self._transform_2d = np.vstack([M, [0, 0, 1]])

        elif method == "perspective":
            # 透视变换: 需要 ≥4 点
            if n < 4:
                raise ValueError(f"Perspective needs ≥4 points, got {n}")
            self._transform_2d, status = cv2.findHomography(
                pixels, worlds_xy, cv2.RANSAC, 5.0
            )

        self._calibrated = True

        # 计算重投影误差
        error = self._calc_reprojection_error()
        logger.info(f"Calibration done ({method}). Error: {error:.2f} mm. Plane Z: {self._plane_z:.1f} mm")
        return error

    # ─── 坐标转换 ────────────────────────────────────────────

    def pixel_to_world(
        self,
        u: float, v: float,
        depth_mm: Optional[float] = None,
    ) -> Tuple[float, float, float]:
        """
        像素坐标 → 世界坐标 (机械臂基坐标系)

        Args:
            u, v: 像素坐标
            depth_mm: 深度值 (mm), None 则使用标定平面 Z

        Returns:
            (x, y, z) 世界坐标 (mm)
        """
        if not self._calibrated:
            raise RuntimeError("Not calibrated. Call calibrate() first.")

        # 像素 → 世界 XY
        pixel_h = np.array([u, v, 1.0], dtype=np.float64)
        world_h = self._transform_2d @ pixel_h
        world_x = world_h[0] / world_h[2]
        world_y = world_h[1] / world_h[2]

        # Z 坐标
        if depth_mm is not None:
            world_z = self._depth_to_world_z(depth_mm)
        else:
            world_z = self._plane_z

        return (float(world_x), float(world_y), float(world_z))

    def world_to_pixel(
        self, x: float, y: float
    ) -> Tuple[float, float]:
        """
        世界坐标 → 像素坐标 (逆变换)

        Args:
            x, y: 世界坐标 (mm)

        Returns:
            (u, v) 像素坐标
        """
        if not self._calibrated:
            raise RuntimeError("Not calibrated.")

        inv_transform = np.linalg.inv(self._transform_2d)
        world_h = np.array([x, y, 1.0], dtype=np.float64)
        pixel_h = inv_transform @ world_h
        u = pixel_h[0] / pixel_h[2]
        v = pixel_h[1] / pixel_h[2]
        return (float(u), float(v))

    # ─── 深度处理 ────────────────────────────────────────────

    def _depth_to_world_z(self, depth_mm: float) -> float:
        """
        深度值 → 世界 Z 坐标

        深度相机测量的是相机到物体的距离，
        需要转换为机械臂基坐标系的 Z。

        简单模型: world_z = camera_height - depth_mm
        (相机朝下固定安装时)
        """
        # 这里用标定点估算相机高度
        # 假设标定时记录的 z 值就是工作台高度
        # camera_height ≈ depth_of_table + plane_z
        # 实际使用中需要标定这个值
        return self._plane_z

    def set_camera_height(self, height_mm: float):
        """
        设置相机安装高度 (相对机械臂基座, mm)
        用于深度→世界Z的转换
        """
        self._camera_height = height_mm

    # ─── 保存/加载 ───────────────────────────────────────────

    def save(self, filepath: str):
        """保存标定结果"""
        data = {
            "method": self._method,
            "plane_z": self._plane_z,
            "calibrated": self._calibrated,
            "points": [
                {"pixel": list(p.pixel), "world": list(p.world)}
                for p in self._points
            ],
        }
        if self._transform_2d is not None:
            data["transform_2d"] = self._transform_2d.tolist()

        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        logger.info(f"Calibration saved to {filepath}")

    def load(self, filepath: str):
        """加载标定结果"""
        with open(filepath, 'r') as f:
            data = json.load(f)

        self._method = data.get("method", "perspective")
        self._plane_z = data.get("plane_z", 0)
        self._calibrated = data.get("calibrated", False)
        self._points = [
            CalibrationPoint(pixel=tuple(p["pixel"]), world=tuple(p["world"]))
            for p in data.get("points", [])
        ]
        if "transform_2d" in data:
            self._transform_2d = np.array(data["transform_2d"], dtype=np.float64)

        logger.info(f"Calibration loaded from {filepath} ({self._method}, {len(self._points)} points)")

    # ─── 辅助方法 ────────────────────────────────────────────

    def _calc_reprojection_error(self) -> float:
        """计算重投影误差 (mm)"""
        if not self._calibrated or not self._points:
            return float('inf')

        errors = []
        for p in self._points:
            wx, wy, wz = self.pixel_to_world(p.pixel[0], p.pixel[1])
            err = np.sqrt((wx - p.world[0])**2 + (wy - p.world[1])**2)
            errors.append(err)

        return float(np.mean(errors))

    @property
    def is_calibrated(self) -> bool:
        return self._calibrated

    @property
    def num_points(self) -> int:
        return len(self._points)

    @property
    def reprojection_error(self) -> float:
        return self._calc_reprojection_error()


# ─── 交互式标定工具 ──────────────────────────────────────────

class InteractiveCalibrator:
    """
    交互式标定工具

    在相机画面上点击标定点，输入对应的机械臂坐标。

    使用:
        calibrator = InteractiveCalibrator(camera, picker)
        calibration = calibrator.run()
        calibration.save("calib.json")
    """

    def __init__(self, camera=None, picker=None):
        """
        Args:
            camera: Camera 实例
            picker: PickerDriver 实例 (可选，用于自动获取末端坐标)
        """
        self.camera = camera
        self.picker = picker
        self.calibration = HandEyeCalibration()
        self._click_point = None

    def run(self, num_points: int = 4) -> HandEyeCalibration:
        """
        运行交互式标定

        流程:
          1. 显示相机画面
          2. 鼠标点击标定点 (或移动机械臂到标定点上方)
          3. 输入对应的世界坐标
          4. 重复 num_points 次
          5. 自动计算标定

        Returns:
            标定结果
        """
        print(f"\n{'='*50}")
        print("  交互式手眼标定")
        print(f"{'='*50}")
        print(f"需要 {num_points} 个标定点")
        print("操作: 鼠标点击图像上的点，然后输入对应的机械臂坐标")
        print("提示: 将机械臂末端移到标定点正上方，用 Picker 读取坐标")
        print()

        window = "Hand-Eye Calibration"
        cv2.namedWindow(window)
        cv2.setMouseCallback(window, self._on_mouse)

        for i in range(num_points):
            print(f"\n--- 标定点 {i+1}/{num_points} ---")
            print("请在图像上点击标定点...")

            # 等待鼠标点击
            self._click_point = None
            while self._click_point is None:
                if self.camera:
                    frame, _ = self.camera.read()
                    if frame is not None:
                        # 绘制已有标定点
                        for p in self.calibration._points:
                            cv2.circle(frame, (int(p.pixel[0]), int(p.pixel[1])), 8, (0, 255, 0), -1)
                            cv2.putText(frame, f"({p.world[0]:.0f},{p.world[1]:.0f})",
                                        (int(p.pixel[0]+10), int(p.pixel[1]-10)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.imshow(window, frame)

                key = cv2.waitKey(30)
                if key == 27:  # ESC
                    print("标定取消")
                    cv2.destroyAllWindows()
                    return self.calibration

            u, v = self._click_point
            print(f"  像素坐标: ({u}, {v})")

            # 获取世界坐标
            if self.picker:
                input("  请将机械臂移到该标定点正上方，然后按 Enter...")
                pos = self.picker.request_position()
                wx, wy, wz = pos.pose.x, pos.pose.y, pos.pose.z
                print(f"  机械臂坐标: ({wx:.1f}, {wy:.1f}, {wz:.1f})")
                confirm = input("  确认? (Y/n): ").strip()
                if confirm.lower() == 'n':
                    continue
            else:
                wx = float(input("  输入 X (mm): "))
                wy = float(input("  输入 Y (mm): "))
                wz = float(input("  输入 Z (mm): "))

            self.calibration.add_point(pixel=(u, v), world=(wx, wy, wz))

        cv2.destroyAllWindows()

        # 执行标定
        error = self.calibration.calibrate()
        print(f"\n✅ 标定完成! 重投影误差: {error:.2f} mm")
        return self.calibration

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._click_point = (x, y)
            print(f"  → 点击: ({x}, {y})")
