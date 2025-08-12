import time
import math
from adafruit_servokit import ServoKit

# ================================
# ⚙️ Khởi tạo ServoKit
kit = ServoKit(channels=16)

# ================================
# 📐 Thông số chân
L1 = 10  # cm
L2 = 10  # cm

# ================================
# 🔧 HÀM IK cho chân phải
def compute_right_leg(x, y):
    D = math.hypot(x, y)
    if D > (L1 + L2):
        raise ValueError("Vượt quá chiều dài chân!")
    theta = math.atan2(y, x)
    cos_beta = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    beta = math.acos(cos_beta)
    alpha_hip = math.degrees(theta - beta)
    cos_gamma = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    gamma = math.acos(cos_gamma)
    alpha_knee = math.degrees(math.pi - gamma)  # ⚠️ theo yêu cầu
    return alpha_knee, alpha_hip

# 🔧 HÀM IK cho chân trái
def compute_left_leg(x, y):
    D = math.hypot(x, y)
    if D > (L1 + L2):
        raise ValueError("Vượt quá chiều dài chân!")
    theta = math.atan2(y, x)
    cos_beta = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    beta = math.acos(cos_beta)
    alpha_hip = math.degrees(theta - beta)
    cos_gamma = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    gamma = math.acos(cos_gamma)
    alpha_knee = math.degrees(math.pi - gamma)
    return alpha_knee, alpha_hip

# ================================
# 🔁 Tổng hợp điều khiển từng kênh
def solve_leg(channel, x, y):
    if channel in (0, 2):  # knee phải
        angle, _ = compute_right_leg(x, y)
    elif channel in (1, 3):  # hip phải
        _, angle = compute_right_leg(x, y)
    elif channel in (4, 6):  # hip trái
        _, angle = compute_left_leg(x, y)
    elif channel in (5, 7):  # knee trái
        angle, _ = compute_left_leg(x, y)
    else:
        raise ValueError("Channel phải từ 0 đến 7")

    # Clamp lại cho an toàn
    angle = max(0, min(180, angle))
    return angle

# ================================
# 🚀 Truyền toàn bộ tọa độ cho 8 servo
def move_all_servos(x, y):
    for ch in range(8):
        angle = solve_leg(ch, x, y)
        kit.servo[ch].angle = angle
        print(f"CH{ch} → {angle:.2f}°")

# ================================
# 🧪 Ví dụ: chân duỗi thẳng xuống tại (0, 18)
move_all_servos(0, 18)
