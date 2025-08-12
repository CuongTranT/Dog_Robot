# dog_xy_apply_chmap.py
# Kênh 1 <- alpha1 (servo1, khâu 1)
# Kênh 0 <- alpha0 + 90° (servo0, khâu 2)

import math
import time
from adafruit_servokit import ServoKit

# ==== CONFIG ====
L1 = 10.0   # Chiều dài khâu 1 (servo1 → khâu đầu tiên)
L2 = 10.0   # Chiều dài khâu 2 (servo0 → khâu thứ hai)

STAND_X = 0.0     # Tọa độ X mục tiêu của điểm G (cm)
STAND_Y = 18.0    # Tọa độ Y mục tiêu của điểm G (cm)
ELBOW_DOWN = True # True = gập xuống như hình

INIT_ANGLES = [100, 180, 110, 180, 70, 20, 85, 20]
MIN_US, MAX_US = 600, 2400

CH0 = 0  # servo0 → điều khiển α0 + 90°
CH1 = 1  # servo1 → điều khiển α1

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def ik_alpha0_alpha1(x, y, elbow_down=True):
    """
    Tính:
      α0: góc toàn cục của khâu thứ 2 (servo0, nối đến điểm G)
      α1: góc toàn cục của khâu thứ 1 (servo1)
    """
    r2 = x*x + y*y
    if r2 < (L1 - L2)**2 - 1e-9 or r2 > (L1 + L2)**2 + 1e-9:
        raise ValueError("Điểm đặt ngoài tầm với")

    c2 = (r2 - L1*L1 - L2*L2) / (2 * L1 * L2)
    c2 = clamp(c2, -1.0, 1.0)
    s2 = math.sqrt(1.0 - c2*c2)
    if not elbow_down:
        s2 = -s2

    q2 = math.atan2(s2, c2)
    alpha1 = math.atan2(y, x) - math.atan2(L2 * s2, L1 + L2 * c2)  # servo1 (khâu 1)
    alpha0 = alpha1 + q2                                           # servo0 (khâu 2)

    return math.degrees(alpha0), math.degrees(alpha1)

def main():
    # Khởi tạo ServoKit
    kit = ServoKit(channels=16)

    # Đặt giới hạn xung cho servo CH0 và CH1
    for ch in (CH0, CH1):
        kit.servo[ch].set_pulse_width_range(MIN_US, MAX_US)

    # Trả các kênh về tư thế khởi tạo
    for ch, ang in enumerate(INIT_ANGLES):
        kit.servo[ch].angle = clamp(ang, 0, 180)
    time.sleep(0.3)

    # Tính IK tại điểm G
    alpha0_deg, alpha1_deg = ik_alpha0_alpha1(STAND_X, STAND_Y, ELBOW_DOWN)

    # Gán ra servo:
    # CH0 <- α0 + 90° → tương đương với (270 - α0) do offset vật lý
    # CH1 <- α1 + 90°
    # ch0_cmd = 270.0 - alpha0_deg
    # ch1_cmd = alpha1_deg + 90.0
    ch0_cmd = 180
    ch1_cmd = 180
    # In kết quả
    print(f"G=({STAND_X}, {STAND_Y}) cm")
    print(f"α0 (khâu 2) = {alpha0_deg:.2f}°,   α1 (khâu 1) = {alpha1_deg:.2f}°")
    print(f"CH0 = α0+90° = {ch0_cmd:.2f}°,   CH1 = α1+90° = {ch1_cmd:.2f}°")

    # Gửi lệnh ra servo
    kit.servo[CH0].angle = clamp(ch0_cmd, 0, 180)
    kit.servo[CH1].angle = clamp(ch1_cmd, 0, 180)

if __name__ == "__main__":
    main()
