from __future__ import division
import math as m
import numpy as np
import time
import Adafruit_PCA9685

# ====================
# ⚙️ CẤU HÌNH SERVO
# ====================
pwm = Adafruit_PCA9685.PCA9685(busnum=1)
pwm.set_pwm_freq(60)  # Hz

# ====================
# 📐 THÔNG SỐ CƠ KHÍ
# ====================
L1 = 100  # mm
L2 = 100  # mm

# Offset kênh servo (nếu cần)
LEFT_OFFSET = np.array([0, 180])  # Góc offset phần hip và knee

# Mapping từ góc độ sang PWM (0–180° → 500–2500μs)
def angle_to_pwm(angle_deg):
    pulse_min = 500
    pulse_max = 2500
    pwm_range = pulse_max - pulse_min
    pulse = pulse_min + (pwm_range * angle_deg / 180.0)
    # Map về 12-bit resolution PWM (0-4096)
    pwm_val = int(pulse * 4096 / 20000)  # 20ms = 50Hz → 20000us
    return pwm_val

# ====================
# 🔧 HÀM ĐỘNG HỌC NGHỊCH
# ====================
def inverse_kinematics(x, y):
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    
    if abs(D) > 1:
        raise ValueError("Vị trí vượt quá giới hạn cơ khí!")
    
    theta2 = -m.atan2(m.sqrt(1 - D**2), D)
    theta1 = m.atan2(
        y * (L1 + L2 * m.cos(theta2)) - x * L2 * m.sin(theta2),
        x * (L1 + L2 * m.cos(theta2)) + y * L2 * m.sin(theta2)
    )
    
    return m.degrees(theta1), m.degrees(theta2)

# ====================
# 🎯 ĐIỀU KHIỂN CHÂN TRÁI
# ====================
def move_left_leg(x, y):
    try:
        theta1, theta2 = inverse_kinematics(x, y)

        print(f"Vị trí P({x}, {y}) => θ1 = {theta1:.2f}°, θ2 = {theta2:.2f}°")

        # Cộng offset nếu cần, tùy cấu hình cơ khí cụ thể
        angle_hip = theta1 + LEFT_OFFSET[0]
        angle_knee = theta2 + LEFT_OFFSET[1]

        # Giới hạn góc hợp lệ (servo MG996R thường 0–180°)
        angle_hip = np.clip(angle_hip, 0, 180)
        angle_knee = np.clip(angle_knee, 0, 180)

        # Đặt PWM
        pwm.set_pwm(0, 0, angle_to_pwm(angle_hip))   # Servo khớp hip – channel 0
        pwm.set_pwm(1, 0, angle_to_pwm(angle_knee))  # Servo khớp gối – channel 1

    except ValueError as e:
        print(f"[❌] Lỗi IK: {e}")

# ====================
# ▶️ TEST CHẠY
# ====================
if __name__ == "__main__":
    # Ví dụ: chân trái chạm đất tại vị trí (x, y) = (100, -100) mm
    move_left_leg(100, -100)
    time.sleep(1)
