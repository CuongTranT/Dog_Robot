import math
import time
import Adafruit_PCA9685

# ========================
# ⚙️ Chiều dài các khâu
# ========================
L1 = 10.0  # cm
L2 = 10.0  # cm

# ========================
# 📐 Tính IK chân phải
# ========================
def compute_theta_right(x, y):
    P = math.hypot(x, y)
    if P > (L1 + L2):
        return None, None, None, None, None, None, False

    cos_sigma = (L1**2 + L2**2 - P**2) / (2 * L1 * L2)
    sigma_rad = math.acos(max(-1, min(1, cos_sigma)))
    sigma_deg = math.degrees(sigma_rad)
    theta2 = 180 - sigma_deg

    beta_rad = math.atan2(y, x)
    beta_deg = math.degrees(beta_rad)
    cos_alpha1 = (L1**2 + P**2 - L2**2) / (2 * L1 * P)
    alpha1_rad = math.acos(max(-1, min(1, cos_alpha1)))
    alpha1_deg = math.degrees(alpha1_rad)
    theta1 = beta_deg - alpha1_deg

    x_k = L1 * math.cos(math.radians(theta1))
    y_k = L1 * math.sin(math.radians(theta1))

    Deg_servo_0 = 180 + theta1   # hip phải
    Deg_servo_1 = theta2 / 2     # knee phải

    return theta1, theta2, Deg_servo_0, Deg_servo_1, x_k, y_k, True

# ========================
# 📐 Tính IK chân trái
# ========================
def compute_theta_left(x, y):
    P = math.hypot(x, y)
    if P > (L1 + L2):
        return None, None, None, None, None, None, False

    cos_sigma = (L1**2 + L2**2 - P**2) / (2 * L1 * L2)
    sigma_rad = math.acos(max(-1, min(1, cos_sigma)))
    sigma_deg = math.degrees(sigma_rad)
    theta2 = 180 - sigma_deg

    beta_rad = math.atan2(y, x)
    beta_deg = math.degrees(beta_rad)
    cos_alpha1 = (L1**2 + P**2 - L2**2) / (2 * L1 * P)
    alpha1_rad = math.acos(max(-1, min(1, cos_alpha1)))
    alpha1_deg = math.degrees(alpha1_rad)
    theta1 = beta_deg - alpha1_deg

    x_k = L1 * math.cos(math.radians(theta1))
    y_k = L1 * math.sin(math.radians(theta1))

    Deg_servo_4 = -theta1        # hip trái (đảo chiều)
    Deg_servo_5 = theta2         # knee trái (giữ nguyên)

    return theta1, theta2, Deg_servo_4, Deg_servo_5, x_k, y_k, True

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
pwm = Adafruit_PCA9685.PCA9685(busnum=1)
pwm.set_pwm_freq(60)  # Hz

# ========================
# 🧪 Nhập tọa độ từ bàn phím
# ========================
def run_interactive():
    while True:
        try:
            leg = input("Nhập chân (L/R hoặc q để thoát): ").strip().lower()
            if leg == "q":
                print("🔚 Thoát.")
                break
            x = float(input("Nhập x (cm): "))
            y = float(input("Nhập y (cm): "))
        except ValueError:
            print("⛔ Sai định dạng.")
            continue

        if leg == "r":
            theta1, theta2, deg0, deg1, xk, yk, ok = compute_theta_right(x, y)
            if not ok:
                print("❌ Điểm ngoài tầm với!")
                continue
            print(f"✔️ Chân phải:")
            print(f"  Servo 0 (hip):  {deg0:.2f}°")
            print(f"  Servo 1 (knee): {deg1:.2f}°")
            set_servo_angle(0, deg0)
            set_servo_angle(1, deg1)

        elif leg == "l":
            theta1, theta2, deg4, deg5, xk, yk, ok = compute_theta_left(x, y)
            if not ok:
                print("❌ Điểm ngoài tầm với!")
                continue
            print(f"✔️ Chân trái:")
            print(f"  Servo 4 (hip):  {deg4:.2f}°")
            print(f"  Servo 5 (knee): {deg5:.2f}°")
            set_servo_angle(4, deg4)
            set_servo_angle(5, deg5)

        else:
            print("❗ Vui lòng nhập 'L' hoặc 'R'.")

# ========================
# ▶️ Gọi chạy
# ========================
run_interactive()
