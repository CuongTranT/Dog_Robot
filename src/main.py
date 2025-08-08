# import time
# import sys
# import termios
# import tty
# import select
# from adafruit_servokit import ServoKit

# kit = ServoKit(channels=8)

# sit_pose = {
#     0: 40, 1: 140,
#     2: 40, 3: 140,
#     4: 40, 5: 140,
#     6: 40, 7: 140
# }

# stand_pose = {
#     0: 90, 1: 95,
#     2: 90, 3: 95,
#     4: 90, 5: 95,
#     6: 90, 7: 95
# }

# def apply_pose(pose_dict):
#     for ch, angle in pose_dict.items():
#         kit.servo[ch].angle = angle
#         print(f"Servo {ch} → {angle}°")
#         time.sleep(0.02)

# def get_key(timeout=0.1):
#     # Đọc phím không blocking từ stdin
#     dr, dw, de = select.select([sys.stdin], [], [], timeout)
#     if dr:
#         return sys.stdin.read(1)
#     return None

# # Cấu hình terminal để đọc 1 ký tự mỗi lần
# old_settings = termios.tcgetattr(sys.stdin)
# tty.setcbreak(sys.stdin.fileno())

# try:
#     print("Nhấn [w] để đứng | [s] để ngồi | [q] để thoát")

#     apply_pose(sit_pose)  # Khởi động ở trạng thái ngồi

#     while True:
#         key = get_key()
#         if key == 'w':
#             print("Chuyển sang tư thế ĐỨNG")
#             apply_pose(stand_pose)
#         elif key == 's':
#             print("Chuyển sang tư thế NGỒI")
#             apply_pose(sit_pose)
#         elif key == 'q':
#             print("Thoát chương trình.")
#             break
#         time.sleep(0.05)

# finally:
#     # Khôi phục cấu hình terminal sau khi kết thúc
#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)



#!/usr/bin/env python3
"""
Code điều khiển servo motor kênh 1
Servo quay từ 0° → 180°, dừng 2 giây, rồi quay ngược về 0°
"""

import time
from adafruit_servokit import ServoKit
import sys
import termios
import tty
import select

kit = ServoKit(channels=16)
channel = 1

# Tăng biên độ điều khiển xung cho MG996R
kit.servo[channel].set_pulse_width_range(min_pulse=500, max_pulse=2500)

for angle in range(0, 181, 10):
    kit.servo[channel].angle = angle
    print(f"Góc hiện tại: {angle}°")
    time.sleep(0.5)
