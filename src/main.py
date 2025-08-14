import math
import time
from Adafruit_PCA9685 import PCA9685

# ==============================
# ⚙️ CẤU HÌNH PCA9685
# ==============================
pwm = PCA9685(busnum=1)
pwm.set_pwm_freq(60)  # MG996R hoạt động tốt ở 60Hz

# ==============================
# 📐 Chuyển góc sang giá trị PWM (12-bit)
# ==============================
def angle_to_pwm(angle_deg):
    pulse_min = 500    # us (0°)
    pulse_max = 2500   # us (180°)
    pulse_us = pulse_min + (pulse_max - pulse_min) * angle_deg / 180
    pulse_len = 1000000.0 / 60 / 4096  # tick time @60Hz
    pwm_val = int(pulse_us / pulse_len)
    return pwm_val

# ==============================
# 🚦 Điều khiển servo
# ==============================
def set_servo_angle(channel, angle_deg):
    angle_deg = max(0, min(180, angle_deg))
    pwm_val = angle_to_pwm(angle_deg)
    pwm.set_pwm(channel, 0, pwm_val)

# ==============================
# 🤖 Hệ IK 2 bậc: tính α1 và α2
# ==============================
def inverse_kinematics_correct(x=0.0, y=10.0, L1=10.0, L2=10.0):
    D = math.hypot(x, y)
    if D > (L1 + L2):
        raise ValueError("⚠️ Điểm ngoài tầm với")

    cos_a2 = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    alpha2 = math.acos(cos_a2)

    beta = math.atan2(-y, x)  # Trục y hướng xuống
    gamma = math.acos((L1**2 + D**2 - L2**2) / (2 * L1 * D))
    alpha1 = beta - gamma

    return math.degrees(alpha1), math.degrees(alpha2)

# ==============================
# 🧩 Mapping thực tế servo
# ==============================
HIP_OFFSET = 90
HIP_SIGN = -1

KNEE_OFFSET = 0
KNEE_SIGN = 1

def move_leg(x, y):
    alpha1_deg, alpha2_deg = inverse_kinematics_correct(x, y)

    hip_angle = HIP_OFFSET + HIP_SIGN * alpha1_deg
    knee_angle = KNEE_OFFSET + KNEE_SIGN * alpha2_deg

    print(f"→ α1 = {alpha1_deg:.2f}°, α2 = {alpha2_deg:.2f}° | servo_hip = {hip_angle:.2f}°, servo_knee = {knee_angle:.2f}°")

    set_servo_angle(4, hip_angle)   # Channel 0: hip
    set_servo_angle(5, knee_angle)  # Channel 1: knee

# ==============================
# 🧪 Test
# ==============================
if __name__ == "__main__":
    try:
        while True:
            move_leg(x=0.0, y=10.0)  # Gập vuông góc
            time.sleep(2)
    except KeyboardInterrupt:
        print("\n⛔ Kết thúc")
