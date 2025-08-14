import math
import time
from Adafruit_PCA9685 import PCA9685

# ===============================
# ⚙️ Khởi tạo PCA9685
pwm = PCA9685(address=0x40, busnum=1)
pwm.set_pwm_freq(60)  # MG996R hoạt động tốt ở 60Hz

# ===============================
# 📐 Chuyển góc độ sang xung PWM
def angle_to_pwm(angle):
    pulse_min = 500    # us (0°)
    pulse_max = 2500   # us (180°)
    pulse_us = pulse_min + (pulse_max - pulse_min) * angle / 180
    pulse_len = 1000000.0 / 60 / 4096  # 60Hz → chu kỳ ~16.66ms
    pulse = int(pulse_us / pulse_len)
    return pulse

# ===============================
# ⚙️ Thông số chân robot
L1 = 10.0  # cm
L2 = 10.0  # cm

def inverse_kinematics(x, y):
    D = math.hypot(x, y)
    if D > (L1 + L2):
        raise ValueError("❌ Vị trí quá xa!")

    # --- Động học ngược
    cos_alpha2 = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    alpha2 = math.acos(cos_alpha2)

    cos_beta = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    beta = math.acos(cos_beta)

    theta = math.atan2(y, x)
    alpha1 = theta - beta

    # Đổi sang độ
    alpha1_deg = math.degrees(alpha1)
    alpha2_deg = math.degrees(alpha2)

    # Mapping theo servo
    servo_hip_angle = 180 - alpha1_deg
    servo_knee_angle =  alpha2_deg

    if not (0 <= servo_hip_angle <= 180):
        raise ValueError(f"❌ Góc hip vượt giới hạn: {servo_hip_angle:.2f}")
    if not (0 <= servo_knee_angle <= 180):
        raise ValueError(f"❌ Góc knee vượt giới hạn: {servo_knee_angle:.2f}")

    return servo_hip_angle, servo_knee_angle

def set_servo_angle(channel, angle_deg):
    pwm_val = angle_to_pwm(angle_deg)
    pwm.set_pwm(channel, 0, pwm_val)

# ===============================
# 🚀 Chạy thử
if __name__ == "__main__":
    try:
        print("🔧 Nhập tọa độ chân cần đến (x, y) [cm]:")
        x = float(input("x = "))
        y = float(input("y = "))

        hip_deg, knee_deg = inverse_kinematics(x, y)

        print(f"\n🎯 Kết quả:")
        print(f" → Kênh 0 (hip):  {hip_deg:.2f}°")
        print(f" → Kênh 1 (knee): {knee_deg:.2f}°")

        set_servo_angle(0, hip_deg)
        set_servo_angle(1, knee_deg)

    except Exception as e:
        print("❗ Lỗi:", e)
