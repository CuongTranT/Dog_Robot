import time
import math
import sys, termios, tty, select
from adafruit_servokit import ServoKit

# ================================
# ⚙️ Khởi tạo ServoKit
kit = ServoKit(channels=16)

# ================================
# 📐 Thông số chân
L1 = 10  # cm
L2 = 10  # cm

# ================================
# 🔧 HÀM IK cho chân phải
def compute_right_leg(x, y):
    D = math.hypot(x, y)
    if D > (L1 + L2):
        raise ValueError("Vượt quá chiều dài chân!")
    theta = math.atan2(y, x)
    cos_beta = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    beta = math.acos(cos_beta)
    # Hip/phần hông (kênh 1,3)
    alpha_hip = math.degrees(theta - beta) + 50
    # Knee/phần gối (kênh 0,2)
    cos_gamma = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    gamma = math.acos(cos_gamma)
    alpha_knee = math.degrees(math.pi - gamma) + 50
    return alpha_knee, alpha_hip  # servo0/2, servo1/3

# 🔧 HÀM IK cho chân trái
def compute_left_leg(x, y):
    D = math.hypot(x, y)
    if D > (L1 + L2):
        raise ValueError("Vượt quá chiều dài chân!")
    theta = math.atan2(y, x)
    cos_beta = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    beta = math.acos(cos_beta)
    alpha_hip = math.degrees(theta - beta)
    cos_gamma = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    gamma = math.acos(cos_gamma)
    alpha_knee = math.degrees(math.pi - gamma)
    return alpha_knee, alpha_hip  # servo5/7, servo4/6

# ================================
# 🔁 Lấy góc cho từng kênh
def solve_leg(channel, x, y):
    if channel in (0, 2):      # knee phải (front/back)
        angle, _ = compute_right_leg(x, y)
    elif channel in (1, 3):    # hip  phải
        _, angle = compute_right_leg(x, y)
    elif channel in (4, 6):    # hip  trái
        _, angle = compute_left_leg(x, y)
    elif channel in (5, 7):    # knee trái
        angle, _ = compute_left_leg(x, y)
    else:
        raise ValueError("Channel phải từ 0 đến 7")
    # Clamp an toàn
    return max(0, min(180, angle))

def target_angles_for_xy(x, y):
    return [solve_leg(ch, x, y) for ch in range(8)]

# ================================
# 🚀 Gửi góc tới servo (nội suy mượt)
def move_servos_to(angles, steps=40, dt=0.02):
    current = []
    for ch in range(8):
        a = kit.servo[ch].angle
        # Nếu None (chưa set lần nào) thì coi như đang ở target để tránh giật
        current.append(a if a is not None else angles[ch])

    for i in range(1, steps + 1):
        t = i / steps
        for ch in range(8):
            val = current[ch] + (angles[ch] - current[ch]) * t
            kit.servo[ch].angle = val
        time.sleep(dt)

    # In góc cuối
    for ch in range(8):
        print(f"CH{ch} → {kit.servo[ch].angle:.2f}°")

# ================================
# ⌨️ Đọc phím không chặn: w/s để đứng/ngồi
def kbhit():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    return dr != []

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

# ================================
# 📍 Các pose
STAND = (0, 18)  # đứng
SIT   = (0, 9)   # ngồi

def main():
    print("Nhấn 'w' = ĐỨNG, 's' = NGỒI, 'q' = THOÁT")
    # Khởi động: ngồi
    move_servos_to(target_angles_for_xy(*SIT), steps=1, dt=0.01)

    while True:
        if not kbhit():
            time.sleep(0.01)
            continue
        c = getch()
        if c.lower() == 'w':
            print("→ ĐỨNG")
            move_servos_to(target_angles_for_xy(*STAND))
        elif c.lower() == 's':
            print("→ NGỒI")
            move_servos_to(target_angles_for_xy(*SIT))
        elif c.lower() == 'q':
            print("Thoát.")
            break

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nNgắt bởi người dùng.")
