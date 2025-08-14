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
# 🔧 Đặt cả hai kênh về 180 độ
def set_both_to_180():
    pwm_180 = angle_to_pwm(180)
    print(f"↪️ Đặt cả CH0 và CH1 về 180° → PWM {pwm_180}")
    pwm.set_pwm(0, 0, pwm_180)
    pwm.set_pwm(1, 0, pwm_180)

# ===============================
# 🧭 Nhập góc CH0 và CH1 thủ công
def set_manual_angles():
    try:
        ang0 = float(input("Nhập góc CH0 (hông): "))
        ang1 = float(input("Nhập góc CH1 (gối): "))
        pwm0 = angle_to_pwm(ang0)
        pwm1 = angle_to_pwm(ang1)
        print(f"🎯 CH0 → {ang0}° → PWM {pwm0}")
        print(f"🎯 CH1 → {ang1}° → PWM {pwm1}")
        pwm.set_pwm(0, 0, pwm0)
        pwm.set_pwm(1, 0, pwm1)
    except ValueError:
        print("⚠️ Góc phải là số thực (float).")
    except Exception as e:
        print("❌ Lỗi khi set góc:", e)

# ===============================
# 📥 Nhập điều khiển từ bàn phím
def main():
    print("📍 MENU:")
    print("   Nhập '1' → CH0 & CH1 về 180°")
    print("   Nhập '2' → Nhập tọa độ x y (VD: 0 12)")
    print("   Nhập '3' → Nhập góc CH0 & CH1 thủ công")
    print("   Nhập 'q' → Thoát")

    while True:
        try:
            inp = input("\nNhập lựa chọn: ")

            if inp.lower() == 'q':
                print("👋 Thoát.")
                break

            elif inp.strip() == '1':
                set_both_to_180()

            elif inp.strip() == '2':
                xy = input("Nhập x y (VD: 0 12): ").strip().split()
                if len(xy) != 2:
                    print("⚠️ Vui lòng nhập đúng định dạng: x y")
                    continue
                x = float(xy[0])
                y = float(xy[1])
                move_leg(x, y)

            elif inp.strip() == '3':
                set_manual_angles()

            else:
                print("⚠️ Vui lòng nhập '1', '2', '3' hoặc 'q'")

        except Exception as e:
            print("❌ Lỗi:", e)

# ===============================
# 🚀 Chạy chương trình
if __name__ == "__main__":
    main()
