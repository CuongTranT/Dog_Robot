import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

# ---------------------------------------------------------
# THAM SỐ BẠN CHỈNH
# ---------------------------------------------------------
# Góc 'đường đỏ' so với phương thẳng đứng đi xuống (độ)
alpha_deg0 = 90.0   # cho knee kênh 0  (cặp A)
alpha_deg2 = 90.0   # cho knee kênh 2  (cặp B)

# Offset & chiều cho từng kênh (độc lập)
# Knee: servo_angle = KNEE_OFFSET + KNEE_SIGN * (-180 + 2*alpha)
KNEE0_OFFSET, KNEE0_SIGN = 90.0, +1   # kênh 0
KNEE2_OFFSET, KNEE2_SIGN = 90.0, +1   # kênh 2

# Hip giữ 0°, nhưng có OFFSET riêng để đúng cơ khí (SIGN chỉ dùng nếu bạn muốn đảo chiều sau này)
HIP1_OFFSET, HIP1_SIGN = 0.0, +1      # kênh 1
HIP3_OFFSET, HIP3_SIGN = 30.0, +1      # kênh 3

# PWM range (chỉnh theo servo của bạn, ví dụ MG996R thường 500..2500 µs)
PULSE_MIN, PULSE_MAX = 500, 2500
# ---------------------------------------------------------

# Kênh
KNEE0, HIP1 = 0, 1   # cặp A
KNEE2, HIP3 = 2, 3   # cặp B

def clamp(x, lo=0.0, hi=180.0): 
    return max(lo, min(hi, x))

def knee_servo_from_alpha(alpha_deg, offset, sign):
    """IK: hip=0°, L1=L2 => theta_knee = -180 + 2*alpha (độ) -> áp offset/sign kênh."""
    theta_knee = -180.0 + 2.0*alpha_deg
    return clamp(offset + sign * theta_knee)

def init_channels():
    for ch in (KNEE0, HIP1, KNEE2, HIP3):
        kit.servo[ch].actuation_range = 180
        kit.servo[ch].set_pulse_width_range(PULSE_MIN, PULSE_MAX)

def apply_pair_A():
    # Hip kênh 1 giữ 0° cơ học -> servo = HIP1_OFFSET (+ SIGN*0)
    kit.servo[HIP1].angle  = clamp(HIP1_OFFSET + HIP1_SIGN * 0.0)
    # Knee kênh 0 theo alpha_deg0
    kit.servo[KNEE0].angle = knee_servo_from_alpha(alpha_deg0, KNEE0_OFFSET, KNEE0_SIGN)
    print(f"A[0,1]: alpha0={alpha_deg0:.1f}° -> knee0={kit.servo[KNEE0].angle:.1f}°, hip1={kit.servo[HIP1].angle:.1f}°")

def apply_pair_B():
    kit.servo[HIP3].angle  = clamp(HIP3_OFFSET + HIP3_SIGN * 0.0)
    kit.servo[KNEE2].angle = knee_servo_from_alpha(alpha_deg2, KNEE2_OFFSET, KNEE2_SIGN)
    print(f"B[2,3]: alpha2={alpha_deg2:.1f}° -> knee2={kit.servo[KNEE2].angle:.1f}°, hip3={kit.servo[HIP3].angle:.1f}°")

if __name__ == "__main__":
    init_channels()
    apply_pair_A()
    time.sleep(0.3)
    apply_pair_B()
