from __future__ import division
import math as m
import numpy as np
import time
import Adafruit_PCA9685

# ====================
# ⚙️ CẤU HÌNH SERVO
# ====================
pwm = Adafruit_PCA9685.PCA9685(busnum=1)
pwm.set_pwm_freq(60)

# Chiều dài khâu
l1 = 100  # mm
l2 = 100  # mm

# Vị trí chân trái
pos_left = np.array([0, 0, 0])
left_leg = 0

# Offset mapping servo (góc cơ học)
LEFT_OFFSET = np.array([180, 225])  # t1_offset, t2_offset

# ====================
# ⚠️ Mapping góc → xung PWM
# ====================
def angle2pulse(angle):
    # mapping từ 0-180° → xung PWM (100-600)
    return int(100 + (500 * angle / 180))

# ====================
# Gửi lệnh điều khiển servo
# ====================
def setLegAngles(leg_address, Theta1, Theta2):
    # Áp dụng offset cho servo trái
    if leg_address == left_leg:
        Theta1 = Theta1 - LEFT_OFFSET[0]  # hip
        Theta2 = Theta2 - LEFT_OFFSET[1]  # knee
    else:
        print("❌ Sai leg_address")
        return

    print(f"[REAL TEST] θ1 = {Theta1:.1f}°, θ2 = {Theta2:.1f}°")

    pwm.set_pwm(leg_address*2, 0, angle2pulse(Theta1))     # channel 0
    pwm.set_pwm(leg_address*2 + 1, 0, angle2pulse(Theta2)) # channel 1

# ====================
# Hàm kiểm tra workspace
# ====================
def Check_work_space(x, y, z):
    k1 = x**2 + y**2 + z**2
    if k1 > (l1 + l2)**2 or k1 < (l1 - l2)**2:
        print("❌ Out of workspace: ", x, y, z)
        exit()

# ====================
# HÀM ĐỘNG HỌC NGHỊCH
# ====================
def LEFT_Inverse_Kinematics(x, y, z):
    global pos_left
    Check_work_space(x, y, z)

    D = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    if abs(D) > 1:
        print("❌ Vượt giới hạn IK: D =", D)
        return

    t2 = -m.atan2(m.sqrt(1 - D**2), D)
    t1 = m.atan2(y*(l1 + l2*m.cos(t2)) - x*l2*m.sin(t2),
                 x*(l1 + l2*m.cos(t2)) + y*l2*m.sin(t2))

    t1_deg = t1 * 180 / np.pi
    t2_deg = t2 * 180 / np.pi

    setLegAngles(left_leg, t1_deg, t2_deg)
    pos_left = [x, y, z]

# ====================
# 📌 TEST THỰC TẾ
# ====================
if __name__ == "__main__":
    print("==== TEST THỰC TẾ LEFT IK ====")

    test_points = [
        (0, 10, 0),    # điểm hợp lệ
    # co tối đa
    ]

    for x, y, z in test_points:
        print(f"\n▶️ Test IK tại ({x}, {y}, {z})")
        LEFT_Inverse_Kinematics(x, y, z)
        time.sleep(2)