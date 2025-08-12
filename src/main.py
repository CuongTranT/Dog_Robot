# dog_xy_apply_chmap_custom_per_servo.py
# CH0 & CH2 ← alpha0
# CH1 & CH3 ← alpha1
# Cho phép hiệu chỉnh riêng từng servo

import math
import time
from adafruit_servokit import ServoKit

# ==== CONFIG ====
L1 = 10.0
L2 = 10.0

STAND_X = 0.0
STAND_Y = 18.0
ELBOW_DOWN = True

INIT_ANGLES = [100, 180, 110, 180, 70, 20, 85, 20]
MIN_US, MAX_US = 600, 2400

CH0 = 0
CH1 = 1
CH2 = 2
CH3 = 3

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def ik_alpha0_alpha1(x, y, elbow_down=True):
    r2 = x*x + y*y
    if r2 < (L1 - L2)**2 - 1e-9 or r2 > (L1 + L2)**2 + 1e-9:
        raise ValueError("Điểm đặt ngoài tầm với")

    c2 = (r2 - L1*L1 - L2*L2) / (2 * L1 * L2)
    c2 = clamp(c2, -1.0, 1.0)
    s2 = math.sqrt(1.0 - c2*c2)
    if not elbow_down:
        s2 = -s2

    q2 = math.atan2(s2, c2)
    alpha1 = math.atan2(y, x) - math.atan2(L2 * s2, L1 + L2 * c2)
    alpha0 = alpha1 + q2

    return math.degrees(alpha0), math.degrees(alpha1)

def main():
    kit = ServoKit(channels=16)

    for ch in (CH0, CH1, CH2, CH3):
        kit.servo[ch].set_pulse_width_range(MIN_US, MAX_US)

    for ch, ang in enumerate(INIT_ANGLES):
        kit.servo[ch].angle = clamp(ang, 0, 180)
    time.sleep(0.3)

    alpha0_deg, alpha1_deg = ik_alpha0_alpha1(STAND_X, STAND_Y, ELBOW_DOWN)

    # Tách riêng từng kênh để hiệu chỉnh độc lập (offset, mirror, sign)
    alpha_servo0 = 250.0 - alpha0_deg   # CH0 → α0 + 90°
    alpha_servo1 = alpha1_deg + 90.0    # CH1 → α1 + 90°
    alpha_servo2 = 270.0 - alpha0_deg   # CH2 → α0 + 90°
    alpha_servo3 = alpha1_deg + 70.0    # CH3 → α1 + 90°

    print(f"G=({STAND_X}, {STAND_Y}) cm")
    print(f"α0 (khâu 2) = {alpha0_deg:.2f}°,   α1 (khâu 1) = {alpha1_deg:.2f}°")
    print(f"CH0 = {alpha_servo0:.2f}°,   CH1 = {alpha_servo1:.2f}°")
    print(f"CH2 = {alpha_servo2:.2f}°,   CH3 = {alpha_servo3:.2f}°")

    kit.servo[CH0].angle = clamp(alpha_servo0, 0, 180)
    kit.servo[CH1].angle = clamp(alpha_servo1, 0, 180)
    kit.servo[CH2].angle = clamp(alpha_servo2, 0, 180)
    kit.servo[CH3].angle = clamp(alpha_servo3, 0, 180)

if __name__ == "__main__":
    main()
