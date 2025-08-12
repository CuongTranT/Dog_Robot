import math
import time
from adafruit_servokit import ServoKit

# ==== CONFIG ====
L1 = 10.0  # Chiều dài khâu 1
L2 = 10.0  # Chiều dài khâu 2
MIN_US, MAX_US = 600, 2400

# Góc khởi tạo cho tất cả các kênh
INIT_ANGLES = [100, 180, 110, 180, 70, 20, 85, 20]

# Tọa độ mục tiêu cho mỗi chân
GOAL_POINTS = {
    "front_right": (0.0, 18.0),
    "back_right":  (0.0, 18.0),
    "front_left":  (0.0, 18.0),
    "back_left":   (0.0, 18.0),
}

# Kênh servo tương ứng từng chân
SERVO_CHANNELS = {
    "front_right": {"hip": 0, "knee": 1},
    "back_right":  {"hip": 2, "knee": 3},
    "front_left":  {"hip": 4, "knee": 5},
    "back_left":   {"hip": 6, "knee": 7},
}

# Mapping chuyển từ alpha → góc servo cụ thể
SERVO_MAP = {
    0: lambda a0, a1: 270 - a0,     # CH0: hip phải
    1: lambda a0, a1: a1 + 90,      # CH1: knee phải
    2: lambda a0, a1: 250 - a0,     # CH2: hip phải sau
    3: lambda a0, a1: a1 + 80,      # CH3: knee phải sau
    4: lambda a0, a1: a0 - 90,      # ✅ CH4: hip trái (sửa lại đúng gốc đối xứng)
    5: lambda a0, a1: a1 + 90,      # CH5: knee trái
    6: lambda a0, a1: a0 - 90,      # CH6: hip trái sau
    7: lambda a0, a1: a1,      # CH7: knee trái sau
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
