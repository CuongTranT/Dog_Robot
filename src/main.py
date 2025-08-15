import math
import time
import Adafruit_PCA9685

# ========================
# ⚙️ Chiều dài các khâu
# ========================
L1 = 10.0  # cm
L2 = 10.0  # cm

# ========================
# 📐 Tính IK + góc servo
# ========================
def compute_theta_angles(x, y):
    P = math.hypot(x, y)

    if P > (L1 + L2):
        return None, None, None, None, None, None, None, False

    cos_sigma = (L1**2 + L2**2 - P**2) / (2 * L1 * L2)
    cos_sigma = max(-1, min(1, cos_sigma))
    sigma_rad = math.acos(cos_sigma)
    sigma_deg = math.degrees(sigma_rad)
    theta2 = 180 - sigma_deg

    beta_rad = math.atan2(y, x)
    beta_deg = math.degrees(beta_rad)
    cos_alpha1 = (L1**2 + P**2 - L2**2) / (2 * L1 * P)
    cos_alpha1 = max(-1, min(1, cos_alpha1))
    alpha1_rad = math.acos(cos_alpha1)
    alpha1_deg = math.degrees(alpha1_rad)
    theta1 = beta_deg - alpha1_deg

    theta1_rad = math.radians(theta1)
    x_k = L1 * math.cos(theta1_rad)
    y_k = L1 * math.sin(theta1_rad)

    Deg_servo_1 = theta2 / 2
    Deg_servo_0 = 180 + theta1

    return theta1, theta2, Deg_servo_0, Deg_servo_1, x_k, y_k, True

# ========================
# 📐 Chuyển độ → PWM
# ========================
def angle_to_pwm(angle_deg):
    pulse_min = 500     # us
    pulse_max = 2500    # us
    pulse_us = pulse_min + (pulse_max - pulse_min) * angle_deg / 180
    pulse_len = 1000000.0 / 60 / 4096  # tick time @60Hz
    pwm_val = int(pulse_us / pulse_len)
    return pwm_val

# ========================
# 🚦 Điều khiển servo
# ========================
def set_servo_angle(channel, angle_deg):
    angle_deg = max(0, min(180, angle_deg))
    pwm_val = angle_to_pwm(angle_deg)
    pwm.set_pwm(channel, 0, pwm_val)

# ========================
# 🔧 Khởi tạo PCA9685
# ========================
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)  # Hz

# ========================
# 🧪 Nhập toạ độ từ bàn phím
# ========================
def run_interactive():
    while True:
        try:
            x = float(input("Nhập x (cm, q để thoát): "))
            y = float(input("Nhập y (cm): "))
        except ValueError:
            print("🔚 Kết thúc chương trình.")
            break

        theta1, theta2, deg0, deg1, xk, yk, ok = compute_theta_angles(x, y)
        if not ok:
            print("❌ Điểm ngoài tầm với!")
            continue

        print(f"✔️ Gửi servo:")
        print(f"  Kênh 0 (hip):  {deg0:.2f}°")
        print(f"  Kênh 1 (knee): {deg1:.2f}°")

        set_servo_angle(0, deg0)
        set_servo_angle(1, deg1)

# ========================
# ▶️ Gọi chạy
# ========================
run_interactive()
