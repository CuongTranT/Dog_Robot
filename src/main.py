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
# 🤖 Hệ IK 2 bậc: tính α1 và α2 (chuẩn theo hình học)
# ==============================
def inverse_kinematics(x, y, L1=10.0, L2=10.0):
    D = math.hypot(x, y)
    if D > (L1 + L2):
        raise ValueError("Điểm ngoài tầm với")

    # alpha2: góc giữa L1 và L2
    cos_alpha2 = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    alpha2 = math.acos(cos_alpha2)  # rad

    # gamma: góc trong tam giác giữa L1 và D
    cos_gamma = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    gamma = math.acos(cos_gamma)

    # alpha1: góc giữa L1 và trục Y (theo chiều ngược kim đồng hồ)
    alpha1 = math.pi - gamma

    return math.degrees(alpha1), math.degrees(alpha2)

# ==============================
# 🦿 Điều khiển servo trực tiếp từ IK
# ==============================
def move_leg(x, y):
    alpha1_deg, alpha2_deg = inverse_kinematics(x, y)

    print(f"→ α1 = {alpha1_deg:.2f}°, α2 = {alpha2_deg:.2f}°")

    # Phải (channel 0-3)
    set_servo_angle(0, 180 - alpha1_deg)  # hip phải trước
    set_servo_angle(1, 90 - alpha2_deg)  # knee phải trước
    set_servo_angle(2, alpha1_deg)  # hip phải sau
    set_servo_angle(3, alpha2_deg)  # knee phải sau

    # Trái (channel 4-5 nếu cần)
    # set_servo_angle(4, 180 - alpha1_deg)
    # set_servo_angle(5, 180 - alpha2_deg)

# ==============================
# 🧪 Test
# ==============================
if __name__ == "__main__":
    try:
        while True:
            move_leg(x=0.0, y=-14.0)  # ví dụ điểm H như hình
            time.sleep(2)
    except KeyboardInterrupt:
        print("\n⛔ Kết thúc")
