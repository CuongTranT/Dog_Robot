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
def inverse_kinematics(x, y, L1=10.0, L2=10.0):
    D = math.hypot(x, y)
    if D > (L1 + L2):
        raise ValueError("Điểm ngoài tầm với")

    # theta2: góc giữa L1 và L2
    cos_theta2 = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    theta2 = math.acos(cos_theta2)

    # theta1
    beta = math.atan2(y, x)
    cos_gamma = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    gamma = math.acos(cos_gamma)
    theta1 = beta - gamma

    return math.degrees(theta1), math.degrees(theta2)

# ==============================
# 🦿 Điều khiển servo trực tiếp từ IK
# ==============================
def move_leg(x, y):
    alpha1_deg, alpha2_deg = inverse_kinematics(x, y)

    print(f"→ α1 = {alpha1_deg:.2f}°, α2 = {alpha2_deg:.2f}°")
    # Trái
    # set_servo_angle(4, 180-alpha1_deg)  # Channel 4: hip
    # set_servo_angle(5, 180-alpha2_deg)  # Channel 5: knee
    # # set_servo_angle(4, 180)  # Channel 4: hip
    # # set_servo_angle(5, 180)  # Channel 5: knee
    # Phải
    set_servo_angle(0, alpha1_deg+8.5)  # Channel 0: hip
    set_servo_angle(1, alpha2_deg+8.5)  # Channel 1: knee
    set_servo_angle(2, alpha1_deg)  # Channel 0: hip
    set_servo_angle(3, alpha2_deg)  # Channel 1: knee
    # set_servo_angle(0, 0)  # Channel 0: hip
    # set_servo_angle(1, 0)  # Channel 1: knee

# ==============================
# 🧪 Test
# ==============================
if __name__ == "__main__":
    try:
        while True:
            move_leg(x=5.0, y=14.0)  # Gập vuông góc
            time.sleep(2)
    except KeyboardInterrupt:
        print("\n⛔ Kết thúc")
