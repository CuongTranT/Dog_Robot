# dog_xy_apply_chmap.py
# Kênh 1 <- alpha2
# Kênh 0 <- alpha1 + 90°

import math, time
from adafruit_servokit import ServoKit

# ---- CONFIG ----
INIT_ANGLES = [100, 180, 110, 180, 70, 20, 85, 20]
L1 = 10.0
L2 = 10.0
STAND_X = 0.0    # sửa tại đây (cm)
STAND_Y = 18.0   # sửa tại đây (cm)
ELBOW_DOWN = True  # đổi False nếu muốn nhánh IK còn lại

MIN_US, MAX_US = 600, 2400
CH0 = 0   # sẽ nhận alpha1 + 90°
CH1 = 1   # sẽ nhận alpha2

def clamp(v, lo, hi): return max(lo, min(hi, v))

def ik_alpha1_alpha2(x, y, elbow_down=True):
    """
    Trả về (alpha1_deg, alpha2_deg) là góc toàn cục của khâu 2 và khâu 1 so với +x.
    Công thức 2R:
      c2 = (x^2+y^2 - L1^2 - L2^2)/(2 L1 L2)
      q2 = atan2(s2, c2),  s2 = ±sqrt(1-c2^2)
      alpha2 = atan2(y,x) - atan2(L2*s2, L1 + L2*c2)
      alpha1 = alpha2 + q2
    """
    r2 = x*x + y*y
    if r2 < (L1-L2)**2 - 1e-9 or r2 > (L1+L2)**2 + 1e-9:
        raise ValueError("Điểm đặt ngoài tầm với")

    c2 = (r2 - L1*L1 - L2*L2) / (2*L1*L2)
    c2 = clamp(c2, -1.0, 1.0)
    s2 = math.sqrt(max(0.0, 1.0 - c2*c2))
    if not elbow_down:
        s2 = -s2

    q2 = math.atan2(s2, c2)
    alpha2 = math.atan2(y, x) - math.atan2(L2*s2, L1 + L2*c2)  # khâu 1
    alpha1 = alpha2 + q2                                       # khâu 2
    return math.degrees(alpha1), math.degrees(alpha2)

def main():
    kit = ServoKit(channels=16)
    # set range và đưa về INIT
    for ch in (CH0, CH1):
        kit.servo[ch].set_pulse_width_range(MIN_US, MAX_US)
    for ch, ang in enumerate(INIT_ANGLES):
        kit.servo[ch].angle = clamp(ang, 0, 180)
    time.sleep(0.3)

    # Tính IK cho (STAND_X, STAND_Y)
    a1, a2 = ik_alpha1_alpha2(STAND_X, STAND_Y, ELBOW_DOWN)

    # Map yêu cầu: CH0 <- a1 + 90°, CH1 <- a2
    ch0_cmd = a1 - 90.0
    ch1_cmd = a2 + 90.0

    print(f"G=({STAND_X},{STAND_Y}) cm -> α1={a1:.2f}°, α2={a2:.2f}°")
    print(f"Set: CH0 = α1+90° = {ch0_cmd:.2f}°,   CH1 = α2 = {ch1_cmd:.2f}°")

    kit.servo[CH0].angle = clamp(ch0_cmd, 0, 180)
    kit.servo[CH1].angle = clamp(ch1_cmd, 0, 180)

if __name__ == "__main__":
    main()
