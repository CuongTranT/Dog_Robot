import time
import math
import sys, termios, tty, select
from adafruit_servokit import ServoKit

# ================================
kit = ServoKit(channels=16)

L1 = 10  # hip length
L2 = 10  # knee length

# Góc cố định khi ngồi (không dùng IK)
INIT_ANGLES = [120, 180, 110, 180, 70, 20, 85, 20]

# ================================
def compute_right_leg(x, y):
    D = math.hypot(x, y)
    if D > (L1 + L2):
        raise ValueError("Điểm vượt quá tầm với!")
    theta = math.atan2(y, x)
    beta = math.acos((L1**2 + D**2 - L2**2) / (2 * L1 * D))
    gamma = math.acos((L1**2 + L2**2 - D**2) / (2 * L1 * L2))
    alpha_hip = math.degrees(theta - beta) + 55
    alpha_knee = math.degrees(math.pi - gamma) + 55
    return alpha_knee, alpha_hip

def compute_left_leg(x, y):
    D = math.hypot(x, y)
    if D > (L1 + L2):
        raise ValueError("Điểm vượt quá tầm với!")
    theta = math.atan2(y, x)
    beta = math.acos((L1**2 + D**2 - L2**2) / (2 * L1 * D))
    gamma = math.acos((L1**2 + L2**2 - D**2) / (2 * L1 * L2))
    alpha_hip = math.degrees(theta - beta)
    alpha_knee = math.degrees(math.pi - gamma)
    return alpha_knee, alpha_hip

def solve_leg(ch, x, y):
    if ch in (0, 2):
        angle, _ = compute_right_leg(x, y)
    elif ch in (1, 3):
        _, angle = compute_right_leg(x, y)
    elif ch in (4, 6):
        _, angle = compute_left_leg(x, y)
    elif ch in (5, 7):
        angle, _ = compute_left_leg(x, y)
    else:
        raise ValueError("Kênh không hợp lệ")
    return max(0, min(180, angle))

def compute_angles_for_xy(x, y):
    return [solve_leg(ch, x, y) for ch in range(8)]

def move_servos_to(angles, steps=40, dt=0.02):
    current = []
    for ch in range(8):
        now = kit.servo[ch].angle
        current.append(now if now is not None else angles[ch])

    for i in range(1, steps + 1):
        t = i / steps
        for ch in range(8):
            val = current[ch] + (angles[ch] - current[ch]) * t
            kit.servo[ch].angle = val
        time.sleep(dt)

    for ch in range(8):
        print(f"CH{ch} → {kit.servo[ch].angle:.2f}°")

# ================================
# Đọc phím không chặn
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
# Pose
STAND = (0, 18)  # dùng IK
SIT_ANGLES = INIT_ANGLES  # dùng góc cứng

def main():
    print("Nhấn 'w' = ĐỨNG, 's' = NGỒI, 'q' = THOÁT")
    # Khởi động ở chế độ NGỒI
    move_servos_to(SIT_ANGLES, steps=1, dt=0.01)

    while True:
        if not kbhit():
            time.sleep(0.01)
            continue
        c = getch().lower()
        if c == 'w':
            print("→ ĐỨNG")
            angles = compute_angles_for_xy(*STAND)
            move_servos_to(angles)
        elif c == 's':
            print("→ NGỒI")
            move_servos_to(SIT_ANGLES)
        elif c == 'q':
            print("Thoát.")
            break

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nNgắt bởi người dùng.")
