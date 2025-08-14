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
# 📐 Tính IK và điều khiển servo
def move_leg(x, y, L1=10, L2=10):
    D = math.hypot(x, y)
    if D > (L1 + L2):
        print("🚫 Điểm nằm ngoài vùng làm việc")
        return

    # GÓC GỐI (theta2)
    cos_a2 = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    alpha2 = math.degrees(math.acos(cos_a2))
    theta2 = 180 - alpha2  # mapping theo hình bạn gửi

    # GÓC HÔNG (theta1)
    cos_a1 = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    alpha1 = math.degrees(math.acos(cos_a1))
    theta1 = 90 + alpha1  # mapping theo sơ đồ trục

    # In kết quả
    print(f"↪️ Góc Hông (CH0): {theta1:.2f}° → PWM {angle_to_pwm(theta1)}")
    print(f"↪️ Góc Gối (CH1): {theta2:.2f}° → PWM {angle_to_pwm(theta2)}")

    # Điều khiển servo
    pwm.set_pwm(0, 0, angle_to_pwm(theta1))  # CH0 = hông
    pwm.set_pwm(1, 0, angle_to_pwm(theta2))  # CH1 = gối

# ===============================
# 📥 Nhập tọa độ từ bàn phím
def main():
    print("📍 Nhập tọa độ (x, y) để điều khiển chân. Gõ 'q' để thoát.")

    while True:
        try:
            inp = input("Nhập x y (vd: 0 12): ")
            if inp.lower() == 'q':
                print("👋 Thoát.")
                break

            parts = inp.strip().split()
            if len(parts) != 2:
                print("⚠️ Vui lòng nhập đúng định dạng: x y")
                continue

            x = float(parts[0])
            y = float(parts[1])
            move_leg(x, y)

        except Exception as e:
            print("❌ Lỗi:", e)

# ===============================
# 🚀 Chạy chương trình
if __name__ == "__main__":
    main()
