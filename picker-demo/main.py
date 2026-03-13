"""
Picker Demo 示例脚本

使用前:
  1. 连接 Picker 到电脑 USB
  2. 找到串口: ls /dev/tty.usb*
  3. 修改下面的 MAIN_PORT 和 NANO_PORT
"""

import time
import logging
from driver.picker_serial import PickerDriver

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

# ============= 修改这里 =============
MAIN_PORT = "/dev/tty.usbmodem_CHANGE_ME"   # Teensy 4.1
NANO_PORT = "/dev/tty.usbmodem_CHANGE_ME_2" # Arduino Nano (舵机扩展板)
# ====================================


def demo_basic_motion():
    """基础运动测试"""
    with PickerDriver(MAIN_PORT, NANO_PORT) as picker:
        # 设置速度
        picker.set_speed(25)  # 25% 速度，安全第一
        picker.set_accel(20)
        picker.set_decel(20)

        # 查询当前位置
        pos = picker.request_position()
        print(f"当前关节: {pos.joints}")
        print(f"当前位姿: {pos.pose}")

        # 关节空间运动 - 回零位
        print("\n>>> 运动到零位...")
        picker.move_joints(0, 0, 0, 0, 0, 0)
        time.sleep(1)

        # 查询新位置
        pos = picker.request_position()
        print(f"零位关节: {pos.joints}")
        print(f"零位位姿: {pos.pose}")

        # 关节空间运动 - J1 转 30 度
        print("\n>>> J1 转到 30°...")
        picker.move_joints(30, 0, 0, 0, 0, 0)
        time.sleep(1)

        # 回零
        print("\n>>> 回零位...")
        picker.move_joints(0, 0, 0, 0, 0, 0)


def demo_cartesian_motion():
    """笛卡尔空间运动测试"""
    with PickerDriver(MAIN_PORT, NANO_PORT) as picker:
        picker.set_speed(20)

        # 先回零
        picker.move_joints(0, 0, 0, 0, 0, 0)
        pos = picker.request_position()
        print(f"起始位姿: {pos.pose}")

        # 笛卡尔运动 - 向上移动 50mm
        print("\n>>> Z + 50mm ...")
        picker.move_cartesian(
            pos.pose.x, pos.pose.y, pos.pose.z + 50,
            pos.pose.rx, pos.pose.ry, pos.pose.rz,
        )

        time.sleep(1)

        # 直线运动回去
        print("\n>>> 直线运动回原位...")
        picker.move_linear(
            pos.pose.x, pos.pose.y, pos.pose.z,
            pos.pose.rx, pos.pose.ry, pos.pose.rz,
        )


def demo_gripper():
    """夹爪测试"""
    with PickerDriver(MAIN_PORT, NANO_PORT) as picker:
        print(">>> 张开夹爪...")
        picker.gripper_open(servo=0, position=90)
        time.sleep(1)

        print(">>> 闭合夹爪...")
        picker.gripper_close(servo=0, position=30)
        time.sleep(1)

        print(">>> 再次张开...")
        picker.gripper_open(servo=0, position=90)


def demo_pick_and_place():
    """
    抓取放置 demo (需要根据实际坐标调整!)

    流程:
      1. 移到抓取位上方
      2. 下降到抓取高度
      3. 夹爪闭合
      4. 抬起
      5. 移到放置位上方
      6. 下降到放置高度
      7. 夹爪张开
      8. 抬起回安全位
    """
    # !! 这些坐标需要根据你的实际工作台调整 !!
    SAFE_Z = 300       # 安全高度 (mm)
    PICK_POS = (200, 100, 50, 0, 0, 0)   # 抓取位 (x,y,z,rx,ry,rz)
    PLACE_POS = (200, -100, 50, 0, 0, 0) # 放置位

    with PickerDriver(MAIN_PORT, NANO_PORT) as picker:
        picker.set_speed(25)

        # 1. 张开夹爪
        picker.gripper_open()
        time.sleep(0.5)

        # 2. 移到抓取位上方
        print(">>> 移到抓取位上方...")
        picker.move_cartesian(PICK_POS[0], PICK_POS[1], SAFE_Z, *PICK_POS[3:])
        time.sleep(0.3)

        # 3. 下降到抓取高度
        print(">>> 下降抓取...")
        picker.move_linear(PICK_POS[0], PICK_POS[1], PICK_POS[2], *PICK_POS[3:])
        time.sleep(0.3)

        # 4. 夹爪闭合
        print(">>> 夹爪闭合...")
        picker.gripper_close()
        time.sleep(0.5)

        # 5. 抬起
        print(">>> 抬起...")
        picker.move_linear(PICK_POS[0], PICK_POS[1], SAFE_Z, *PICK_POS[3:])
        time.sleep(0.3)

        # 6. 移到放置位上方
        print(">>> 移到放置位...")
        picker.move_cartesian(PLACE_POS[0], PLACE_POS[1], SAFE_Z, *PLACE_POS[3:])
        time.sleep(0.3)

        # 7. 下降到放置高度
        print(">>> 下降放置...")
        picker.move_linear(PLACE_POS[0], PLACE_POS[1], PLACE_POS[2], *PLACE_POS[3:])
        time.sleep(0.3)

        # 8. 张开夹爪
        print(">>> 夹爪张开...")
        picker.gripper_open()
        time.sleep(0.5)

        # 9. 抬起回安全位
        print(">>> 回安全位...")
        picker.move_linear(PLACE_POS[0], PLACE_POS[1], SAFE_Z, *PLACE_POS[3:])

        print("✅ 抓取放置完成!")


if __name__ == "__main__":
    print("=" * 50)
    print("  Picker V2 Demo")
    print("=" * 50)
    print()
    print("选择测试:")
    print("  1. 基础关节运动")
    print("  2. 笛卡尔空间运动")
    print("  3. 夹爪测试")
    print("  4. 抓取放置 (需调参)")
    print()

    choice = input("输入编号 (1-4): ").strip()

    if choice == "1":
        demo_basic_motion()
    elif choice == "2":
        demo_cartesian_motion()
    elif choice == "3":
        demo_gripper()
    elif choice == "4":
        demo_pick_and_place()
    else:
        print("无效选择")
