import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

# =========================================================
# BÊN TRÁI (0..3) — giữ nguyên kiểu bạn đang dùng
# =========================================================
alpha_deg0 = 90.0   # knee CH0 (pair A)
alpha_deg2 = 80.0   # knee CH2 (pair B)

KNEE0_OFFSET, KNEE0_SIGN = 90.0, +1   # CH0
KNEE2_OFFSET, KNEE2_SIGN = 90.0, +1   # CH2
HIP1_OFFSET,  HIP1_SIGN  = 0.0,  +1   # CH1
HIP3_OFFSET,  HIP3_SIGN  = 10.0, +1   # CH3

# Nếu muốn thay đổi góc mục tiêu hip (thay vì luôn 0°), chỉnh các TARGET dưới đây
HIP1_TARGET = 0.0   # deg, mục tiêu cơ học cho CH1
HIP3_TARGET = 0.0   # deg, mục tiêu cơ học cho CH3

# =========================================================
# BÊN PHẢI (4..7) — TÙY CHỈNH ĐỘC LẬP (không mirror)
# =========================================================
alpha_deg4 = 90.0   # knee CH4 (pair C)
alpha_deg6 = 80.0   # knee CH6 (pair D)

KNEE4_OFFSET, KNEE4_SIGN = 180.0, +1   # CH4
KNEE6_OFFSET, KNEE6_SIGN = 90.0, +1   # CH6
HIP5_OFFSET,  HIP5_SIGN  = 0.0,  +1   # CH5
HIP7_OFFSET,  HIP7_SIGN  = 0.0,  +1   # CH7

HIP5_TARGET = 90.0   # deg, mục tiêu cơ học cho CH5
HIP7_TARGET = 90.0   # deg, mục tiêu cơ học cho CH7

# =========================================================
# PWM range (MG996R thường 500..2500 µs; chỉnh theo servo của bạn)
PULSE_MIN, PULSE_MAX = 500, 2500

# Kênh
KNEE0, HIP1 = 0, 1
KNEE2, HIP3 = 2, 3
KNEE4, HIP5 = 4, 5
KNEE6, HIP7 = 6, 7

def clamp(x, lo=0.0, hi=180.0):
    return max(lo, min(hi, x))

def knee_servo_from_alpha(alpha_deg, offset, sign):
    """
    IK cho cơ cấu 'đường đỏ' với hip=0°, L1=L2:
      theta_knee (deg) = -180 + 2*alpha
    Sau đó áp offset/sign riêng cho từng kênh knee.
    """
    theta_knee = -180.0 + 2.0*alpha_deg
    return clamp(offset + sign * theta_knee)

def hip_servo_from_target(target_deg, offset, sign):
    """Hip giữ target_deg (thường 0°), rồi offset/sign từng kênh."""
    return clamp(offset + sign * target_deg)

def init_channels():
    for ch in (KNEE0, HIP1, KNEE2, HIP3, KNEE4, HIP5, KNEE6, HIP7):
        kit.servo[ch].actuation_range = 180
        kit.servo[ch].set_pulse_width_range(PULSE_MIN, PULSE_MAX)

def apply_pair(knee_ch, hip_ch, alpha_deg, knee_off, knee_sign, hip_target, hip_off, hip_sign, label):
    knee = knee_servo_from_alpha(alpha_deg, knee_off, knee_sign)
    hip  = hip_servo_from_target(hip_target, hip_off, hip_sign)
    kit.servo[knee_ch].angle = knee
    kit.servo[hip_ch].angle  = hip
    print(f"{label}[{knee_ch},{hip_ch}]  alpha={alpha_deg:.1f}°  -> knee={knee:.1f}°, hip={hip:.1f}°")

if __name__ == "__main__":
    init_channels()

    # BÊN TRÁI (A,B)
    apply_pair(KNEE0, HIP1, alpha_deg0, KNEE0_OFFSET, KNEE0_SIGN, HIP1_TARGET, HIP1_OFFSET, HIP1_SIGN, "A")
    time.sleep(0.2)
    apply_pair(KNEE2, HIP3, alpha_deg2, KNEE2_OFFSET, KNEE2_SIGN, HIP3_TARGET, HIP3_OFFSET, HIP3_SIGN, "B")

    # BÊN PHẢI (C,D) — độc lập, KHÔNG mirror
    time.sleep(0.2)
    apply_pair(KNEE4, HIP5, alpha_deg4, KNEE4_OFFSET, KNEE4_SIGN, HIP5_TARGET, HIP5_OFFSET, HIP5_SIGN, "C")
    time.sleep(0.2)
    apply_pair(KNEE6, HIP7, alpha_deg6, KNEE6_OFFSET, KNEE6_SIGN, HIP7_TARGET, HIP7_OFFSET, HIP7_SIGN, "D")
