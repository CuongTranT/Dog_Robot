import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

# ---------------------------------------------------------
# BÊN TRÁI (giữ nguyên như bạn đã chỉnh)
# ---------------------------------------------------------
alpha_deg0 = 90.0   # knee CH0 (pair A)
alpha_deg2 = 80.0   # knee CH2 (pair B)

# Offset & chiều từng kênh trái
KNEE0_OFFSET, KNEE0_SIGN = 90.0, +1   # CH0
KNEE2_OFFSET, KNEE2_SIGN = 90.0, +1   # CH2
HIP1_OFFSET,  HIP1_SIGN  = 0.0,  +1   # CH1
HIP3_OFFSET,  HIP3_SIGN  = 10.0, +1   # CH3

# PWM range
PULSE_MIN, PULSE_MAX = 500, 2500

# Kênh trái
KNEE0, HIP1 = 0, 1     # pair A
KNEE2, HIP3 = 2, 3     # pair B

# ---------------------------------------------------------
# BÊN PHẢI (mặc định ngược góc so với trái)
# ---------------------------------------------------------
RIGHT_INVERT = True     # True => right = 180 - left ; False => copy y nguyên

# Offset & chiều RIÊNG cho từng kênh phải (điều chỉnh độc lập nếu cần)
# Mặc định khởi tạo theo kênh trái tương ứng để dễ bắt đầu.
KNEE4_OFFSET, KNEE4_SIGN = KNEE0_OFFSET, +1   # CH4 tương ứng CH0
HIP5_OFFSET,  HIP5_SIGN  = HIP1_OFFSET,  +1   # CH5 tương ứng CH1
KNEE6_OFFSET, KNEE6_SIGN = KNEE2_OFFSET, +1   # CH6 tương ứng CH2
HIP7_OFFSET,  HIP7_SIGN  = HIP3_OFFSET,  +1   # CH7 tương ứng CH3

# Kênh phải
KNEE4, HIP5 = 4, 5
KNEE6, HIP7 = 6, 7

# ---------------------------------------------------------

def clamp(x, lo=0.0, hi=180.0):
    return max(lo, min(hi, x))

def knee_servo_from_alpha(alpha_deg, offset, sign):
    # IK: hip=0°, L1=L2 => theta_knee = -180 + 2*alpha (độ)
    theta_knee = -180.0 + 2.0 * alpha_deg
    return clamp(offset + sign * theta_knee)

def init_channels():
    for ch in (KNEE0, HIP1, KNEE2, HIP3, KNEE4, HIP5, KNEE6, HIP7):
        kit.servo[ch].actuation_range = 180
        kit.servo[ch].set_pulse_width_range(PULSE_MIN, PULSE_MAX)

def apply_left_pairs():
    # Hip trái giữ 0°
    hip1 = clamp(HIP1_OFFSET + HIP1_SIGN * 0.0)
    hip3 = clamp(HIP3_OFFSET + HIP3_SIGN * 0.0)
    kit.servo[HIP1].angle, kit.servo[HIP3].angle = hip1, hip3

    # Knee trái theo alpha
    knee0 = knee_servo_from_alpha(alpha_deg0, KNEE0_OFFSET, KNEE0_SIGN)
    knee2 = knee_servo_from_alpha(alpha_deg2, KNEE2_OFFSET, KNEE2_SIGN)
    kit.servo[KNEE0].angle, kit.servo[KNEE2].angle = knee0, knee2

    return (knee0, hip1, knee2, hip3)

def mirror_to_right(left_angle, offset, sign):
    val = (180.0 - left_angle) if RIGHT_INVERT else left_angle
    return clamp(offset + sign * val)

def apply_right_from_left(knee0, hip1, knee2, hip3):
    # Map: 4<-0, 5<-1, 6<-2, 7<-3
    kit.servo[KNEE4].angle = mirror_to_right(knee0, KNEE4_OFFSET, KNEE4_SIGN)
    kit.servo[HIP5].angle  = mirror_to_right(hip1,  HIP5_OFFSET,  HIP5_SIGN)
    kit.servo[KNEE6].angle = mirror_to_right(knee2, KNEE6_OFFSET, KNEE6_SIGN)
    kit.servo[HIP7].angle  = mirror_to_right(hip3,  HIP7_OFFSET,  HIP7_SIGN)

if __name__ == "__main__":
    init_channels()

    # Áp bên trái trước…
    k0, h1, k2, h3 = apply_left_pairs()
    print(f"LEFT  -> CH0={k0:.1f}°, CH1={h1:.1f}° | CH2={k2:.1f}°, CH3={h3:.1f}°")

    # …rồi phản chiếu sang bên phải
    apply_right_from_left(k0, h1, k2, h3)
    print("RIGHT -> CH4,CH5,CH6,CH7 đã set (ngược góc)" if RIGHT_INVERT else
          "RIGHT -> CH4,CH5,CH6,CH7 đã copy y nguyên")
