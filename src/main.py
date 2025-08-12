import math
import time
from adafruit_servokit import ServoKit

# ==== CONFIG ====
L1 = 10.0  # Chiều dài khâu 1
L2 = 10.0  # Chiều dài khâu 2
MIN_US, MAX_US = 600, 2400

# Góc khởi tạo cho tất cả các kênh
INIT_ANGLES = [100, 180, 110, 180, 70, 20, 85, 20]

# Tọa độ mục tiêu cho mỗi chân (tư thế đứng trung tính)
# Định danh theo yêu cầu: A(2,3), B(6,7), C(0,1), D(4,5)
GOAL_POINTS = {
    "A": (0.0, 18.0),  # kênh 2,3
    "B": (0.0, 18.0),  # kênh 6,7
    "C": (0.0, 18.0),  # kênh 0,1
    "D": (0.0, 18.0),  # kênh 4,5
}

# Kênh servo tương ứng từng chân theo A/B/C/D
SERVO_CHANNELS = {
    "A": {"hip": 2, "knee": 3},  # A: kênh 2,3
    "B": {"hip": 6, "knee": 7},  # B: kênh 6,7
    "C": {"hip": 0, "knee": 1},  # C: kênh 0,1
    "D": {"hip": 4, "knee": 5},  # D: kênh 4,5
}

# Hiệu chỉnh offset từng kênh (độ) để căn tư thế đứng chuẩn
# Có thể tinh chỉnh nhanh tại đây nếu cơ khí lắp lệch
SERVO_OFFSETS = {
    0: 0,    # CH0  hip phải trước (C)
    1: 0,    # CH1  knee phải trước (C)
    2: -20,  # CH2  hip phải sau   (A) — bù tương đương map cũ 250 - a0
    3: -10,  # CH3  knee phải sau  (A) — bù tương đương map cũ a1 + 80
    4: 0,    # CH4  hip trái trước (D)
    5: 0,    # CH5  knee trái trước (D)
    6: 0,    # CH6  hip trái sau   (B)
    7: 0,    # CH7  knee trái sau  (B)
}

def _hip_right(a0_deg, _a1_deg):
    return 270 - a0_deg

def _hip_left(a0_deg, _a1_deg):
    return a0_deg - 90

def _hip_left_reverse(a0_deg, _a1_deg):
    # Chân B và D (bên trái) có gốc tọa độ ngược chiều với A và C
    return 180 - (a0_deg - 90)

def _knee_common(_a0_deg, a1_deg):
    return a1_deg + 90

# Mapping chuyển từ alpha → góc servo cụ thể (đồng nhất trái/phải, có offset tinh chỉnh)
SERVO_MAP = {
    0: lambda a0, a1: _hip_right(a0, a1) + SERVO_OFFSETS[0],      # CH0: hip phải trước (C)
    1: lambda a0, a1: _knee_common(a0, a1) + SERVO_OFFSETS[1],     # CH1: knee phải trước (C)
    2: lambda a0, a1: _hip_right(a0, a1) + SERVO_OFFSETS[2],       # CH2: hip phải sau (A)
    3: lambda a0, a1: _knee_common(a0, a1) + SERVO_OFFSETS[3],     # CH3: knee phải sau (A)
    4: lambda a0, a1: _hip_left_reverse(a0, a1) + SERVO_OFFSETS[4], # CH4: hip trái trước (D) - ngược chiều
    5: lambda a0, a1: _knee_common(a0, a1) + SERVO_OFFSETS[5],     # CH5: knee trái trước (D)
    6: lambda a0, a1: _hip_left_reverse(a0, a1) + SERVO_OFFSETS[6], # CH6: hip trái sau (B) - ngược chiều
    7: lambda a0, a1: _knee_common(a0, a1) + SERVO_OFFSETS[7],     # CH7: knee trái sau (B)
}
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def ik_2d(x, y, elbow_down=True):
    """Tính động học ngược cho tay máy 2 khâu"""
    r2 = x*x + y*y
    if r2 < (L1 - L2)**2 - 1e-9 or r2 > (L1 + L2)**2 + 1e-9:
        raise ValueError("Điểm đặt ngoài tầm với")

    c2 = (r2 - L1*L1 - L2*L2) / (2 * L1 * L2)
    c2 = clamp(c2, -1.0, 1.0)
    s2 = math.sqrt(1.0 - c2*c2)
    if not elbow_down:
        s2 = -s2

    q2 = math.atan2(s2, c2)
    alpha1 = math.atan2(y, x) - math.atan2(L2 * s2, L1 + L2 * c2)  # khâu 1
    alpha0 = alpha1 + q2                                           # khâu 2
    return math.degrees(alpha0), math.degrees(alpha1)

def move_leg(kit, leg_name, x, y, elbow_down=True):
    alpha0, alpha1 = ik_2d(x, y, elbow_down)
    hip_ch  = SERVO_CHANNELS[leg_name]["hip"]
    knee_ch = SERVO_CHANNELS[leg_name]["knee"]

    angle_hip  = clamp(SERVO_MAP[hip_ch](alpha0, alpha1), 0, 180)
    angle_knee = clamp(SERVO_MAP[knee_ch](alpha0, alpha1), 0, 180)

    kit.servo[hip_ch].angle  = angle_hip
    kit.servo[knee_ch].angle = angle_knee

    print(f"[{leg_name.upper()}] G=({x}, {y}) cm")
    print(f"  α0 = {alpha0:.2f}°, α1 = {alpha1:.2f}°")
    print(f"  Servo CH{hip_ch} = {angle_hip:.2f}°, CH{knee_ch} = {angle_knee:.2f}°")

def main():
    kit = ServoKit(channels=16)

    # Cấu hình và đặt tư thế ban đầu
    for ch in range(8):
        kit.servo[ch].set_pulse_width_range(MIN_US, MAX_US)
        kit.servo[ch].angle = clamp(INIT_ANGLES[ch], 0, 180)
    time.sleep(0.5)

    # Điều khiển từng chân về vị trí mục tiêu
    for leg, (x, y) in GOAL_POINTS.items():
        move_leg(kit, leg, x, y, elbow_down=True)

if __name__ == "__main__":
    main()
