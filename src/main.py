import math
import time
import Adafruit_PCA9685
from mpu6050 import mpu6050

# ========================
#  Khai báo chân GPIO / cảm biến
# ========================
TRIG = 23
ECHO = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
sensor = mpu6050(0x68)

# ========================
# Chiều dài các khâu
# ========================
L1 = 10.0  # cm
L2 = 10.0  # cm

# ========================
#   Tính IK chân phải
# ========================
def compute_theta_right(x, y,):

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

    Deg_hip_p = 180 + theta1
    Deg_knee_p = theta2 / 2

    return theta1, theta2, Deg_hip_p , Deg_knee_p , x_k, y_k, True

# ========================
#   Tính IK chân trái
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

    Deg_hip = -theta1
    Deg_knee = theta2

    return theta1, theta2, Deg_hip, Deg_knee, x_k, y_k, True

# ========================
#    Chuyển độ → PWM
# ========================
def angle_to_pwm(angle_deg):
    pulse_min = 500     # us
    pulse_max = 2500    # us
    pulse_us = pulse_min + (pulse_max - pulse_min) * angle_deg / 180
    pulse_len = 1000000.0 / 60 / 4096  # tick time @60Hz
    pwm_val = int(pulse_us / pulse_len)
    return pwm_val

# ========================
#    Điều khiển servo
# ========================
def set_servo_angle(channel, angle_deg, w, b):
    angle_deg = max(0, min(180, angle_deg))
    pwm_val =  angle_to_pwm(w *angle_deg + b)
    pwm.set_pwm(channel, 0, pwm_val)

# ========================
#    Khởi tạo PCA9685
# ========================
pwm = Adafruit_PCA9685.PCA9685(busnum=1)
pwm.set_pwm_freq(60)

# ========================
#  Điều khiển toàn bộ chân
# ========================
def move_all_legs(pos_list):
    # pos_list = [ (x_RF, y_RF), (x_RR, y_RR), (x_LF, y_LF), (x_LR, y_LR) ]
    
    # === Chân phải sau (RR) ===
    x, y = pos_list[0]
    _, _, deg_hip_p, deg_knee_p, _, _, ok = compute_theta_right(x, y)
    if ok:
        # print(f" RF: Hip={deg_hip:.1f}°, Knee={deg_knee:.1f}°")
        set_servo_angle(0, deg_hip_p , 0.786821, 6.395377)
        set_servo_angle(1, deg_knee_p , -1.34892,  120.9353 )
    else:
        print(" RF: Ngoài tầm với")

    # === Chân phải sau (RF) ===
    x, y = pos_list[1]
    _, _, deg_hip_p, deg_knee_p, _, _, ok = compute_theta_right(x, y)
    if ok:
        # print(f" RR: Hip={deg_hip:.1f}°, Knee={deg_knee:.1f}°")
        set_servo_angle(2, deg_hip_p, 1, 0)
        set_servo_angle(3, deg_knee_p,-1.34892, 115.9353) 
        print( "hip1_p", deg_hip_p, "knee1", deg_knee_p)
    else:
        print(" RR: Ngoài tầm với")

    # === Chân trái trước (LF) ===
    x, y = pos_list[2]
    theta1, theta2, deg_hip, deg_knee, _, _, ok = compute_theta_left(x, y)
    if ok:
        # print(f"✔️ LF: Hip={deg_hip:.1f}°, Knee={deg_knee:.1f}°")
        set_servo_angle(4, deg_hip, 1, 0)
        set_servo_angle(5, deg_knee, 1, 0)
        print( "hip1_T", deg_hip, "knee1", deg_knee)
    else:
        print(" LF: Ngoài tầm với")

    # === Chân trái sau (LR) ===
    x, y = pos_list[3]
    theta1, theta2, deg_hip, deg_knee, _, _, ok = compute_theta_left(x, y)
    if ok:
        # print(f"✔️ LR: Hip={deg_hip:.1f}°, Knee={deg_knee:.1f}°")
        set_servo_angle(6, deg_hip, 1.122911, 13.4366)
        set_servo_angle(7, deg_knee, 1.56205, -42.446)
    else:
        print(" LR: Ngoài tầm với")

# ========================
#   Đo khoảng cách
# ========================
def get_distance():
    GPIO.output(TRIG, False)
    time.sleep(0.05)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    duration = pulse_end - pulse_start
    distance = duration * 34300 / 2
    return round(distance, 2)

# ========================
#   Đo góc nghiêng 
# ========================
def get_tilt_angle():
    accel = sensor.get_accel_data()
    ax, ay, az = accel['x'], accel['y'], accel['z']

    # Tính góc nghiêng roll và pitch từ gia tốc
    pitch = math.atan2(ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi
    roll  = math.atan2(ay, math.sqrt(ax**2 + az**2)) * 180 / math.pi

    return round(pitch, 2), round(roll, 2)


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

# ========================
# 🧱 Các dáng chân
# ========================
start_pose = [(0, -18)] * 4   # Dáng khởi động, input 1 5,-15
sit_pose   = [(8, -8)]  * 4   # Dáng ngồi, input 2
stand_pose = [(0, -16)] * 4  # Dáng đứng
walk_pose = [(0, -10), (10, -10), (2, -10), (3, -10)]  # Dáng đi
# ========================
# ▶️ Vòng lặp điều khiển
# ========================
def main():
    print("🤖 Robot khởi động về tư thế ban đầu...")
    move_all_legs(start_pose)

    try:
        while True:
            print("\n nhấn [w=Đứng, s=Ngồi, x=Đi tới, q=Thoát]")
            key = get_key()

            # Kiểm tra nghiêng trước khi thực hiện hành động
            pitch, roll = get_tilt_angle()
            if abs(pitch) > 20 or abs(roll) > 20:
                print(f" Góc nghiêng lớn! (Pitch={pitch}, Roll={roll}) ➤ Dừng để cân bằng.")
                move_all_legs(start_pose)
                continue

            # Kiểm tra vật cản
            distance = get_distance()
            if distance < 10:
                print(f" Vật cản phát hiện ở {distance} cm ➤ Dừng!")
                move_all_legs(start_pose)
                continue

            # Điều khiển theo phím
            if key == 'w':
                print(" Tư thế: Đứng")
                move_all_legs(walk_pose)
            elif key == 's':
                print(" Tư thế: Ngồi")
                move_all_legs(sit_pose)
            elif key == 'x':
                print(" Di chuyển về phía trước...")
                move_all_legs(walk_pose)
            elif key == 'q':
                print(" Thoát chương trình")
                break
            else:
                print(" Nhập lại")

    except KeyboardInterrupt:
        print("\n Thoát chương trình với  Ctrl+C")

    finally:
        GPIO.cleanup()

# ========== CHẠY CHÍNH ==========
if __name__ == "__main__":
    main()

# if __name__ == "__main__":
#     print("Đang đưa robot về vị trí khởi động...")  # 🟢 Tự động chuyển về start_pose
#     move_all_legs(start_pose)
    # while True:
    #     cmd = input("Nhấn (w=đứng, s=ngồi, x=bắt đầu, q=thoát): ").strip().lower()
    #     if cmd == "q":
    #         print("👋 Kết thúc.")
    #         break
    #     elif cmd == "s":
    #         print("🪑 Đang chuyển sang dáng ngồi...")
    #         move_all_legs(sit_pose)
    #     elif cmd == "w":
    #         print("📏 Đang đứng lên...")
    #         move_all_legs(stand_pose)
    #     elif cmd == "x":
    #         print("🟢 Đưa về vị trí bắt đầu...")
    #         move_all_legs(start_pose)
    #     else:
    #         print("❗ Lệnh không hợp lệ. Dùng: w / s / x / q.")

# if __name__ == "__main__":
#     while True:
#         try:
#             print("\n📟 Điều khiển servo thủ công:")
#             ch = int(input("🔘 Nhập kênh servo (0–15, -1 để thoát): "))
#             if ch == -1:
#                 print("👋 Kết thúc chương trình.")
#                 break
#             if not 0 <= ch <= 15:
#                 print("❌ Kênh không hợp lệ! Chọn từ 0 đến 15.")
#                 continue

#             angle = float(input("🎯 Nhập góc (0–180 độ): "))
#             if not 0 <= angle <= 180:
#                 print("❌ Góc không hợp lệ! Nhập từ 0 đến 180.")
#                 continue

#             set_servo_angle(ch, angle, 1, 0)  # Mặc định w=1, b=0
#             print(f"✅ Đã điều khiển servo kênh {ch} đến {angle:.1f}°")

#         except ValueError:
#             print("⚠️ Nhập sai định dạng! Hãy thử lại.")

# if __name__ == "__main__":
#     print("🚀 Điều khiển tất cả chân robot với cùng toạ độ")

#     try:
#         x = float(input("Nhập X (cm): "))
#         y = float(input("Nhập Y (cm): "))

#         # pos_list gồm 4 chân: [RF, RR, LF, LR]
#         pos_list = [(x, y)] * 4
#         move_all_legs(pos_list)

#     except Exception as e:
#         print("⚠️ Lỗi nhập dữ liệu:", e)

