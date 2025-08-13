import Adafruit_PCA9685

# ====================
# ⚙️ CẤU HÌNH PWM
# ====================
pwm = Adafruit_PCA9685.PCA9685(busnum=1)
pwm.set_pwm_freq(60)

# ====================
# ⚠️ Hàm chuyển đổi góc thành xung PWM
# ====================
def angle2pulse(angle):
    # Chuyển góc 0-180° thành giá trị PWM 100 - 600
    return int(100 + (500 * angle / 180))

# ====================
# 🔧 Hàm set kênh 0 quay góc 45 độ
# ====================
def set_servo_channel_0_to_45():
    angle = 180 - 45  # độ
    pulse = angle2pulse(angle)
    pwm.set_pwm(0, 0, pulse)
    print(f"✅ Đã set kênh 0 quay tới {angle}° (PWM: {pulse})")

# ====================
# 📌 Gọi hàm test
# ====================
if __name__ == "__main__":
    set_servo_channel_0_to_45()
