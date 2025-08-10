# dog_start_and_stand.py
# - Khởi động: set góc INIT_ANGLES cho 8 kênh (0..7)
# - Nhấn 'w': tính IK (L1=L2=10cm, bàn chân ngay dưới hông ở cao H) -> tư thế đứng
# - Nhấn 's': về tư thế bắt đầu; 'q': thoát

import sys, termios, tty, select, math, time
from adafruit_servokit import ServoKit

# ---------- CONFIG ----------
INIT_ANGLES = [100, 180, 110, 180, 70, 20, 85, 20]  # tư thế bắt đầu của bạn
L1 = 10.0  # cm (hip)
L2 = 10.0  # cm (knee)
H  = 10.0  # cm: chiều cao “đường chấm gạch” (hông -> đất) khi đứng

MIN_US, MAX_US = 600, 2400
HIP_CH   = [0, 2, 4, 6]  # các kênh khớp hip
KNEE_CH  = [1, 3, 5, 7]  # các kênh khớp knee

# Offset/Sign để quy đổi góc IK -> góc servo (chỉnh theo cơ khí thực tế)
HIP_OFF  = [90.0, 90.0, 90.0, 90.0]
HIP_SIGN = [+1,   +1,   +1,   +1  ]
KNEE_OFF = [90.0, 90.0, 90.0, 90.0]
KNEE_SIGN= [+1,   +1,   +1,   +1  ]
# ----------------------------

def clamp(a, lo=0.0, hi=180.0):
    return max(lo, min(hi, float(a)))

def ik_under_hip(H):
    # IK 2R: x=0, y=-H
    r2 = H*H
    c2 = (r2 - L1*L1 - L2*L2) / (2*L1*L2)
    if not -1.0 <= c2 <= 1.0:
        raise ValueError("Mục tiêu ngoài tầm với (0 < H ≤ L1+L2).")
    th2 = -math.acos(c2)  # chọn nghiệm knee-down
    th1 = -math.pi/2 - math.atan2(L2*math.sin(th2), L1 + L2*math.cos(th2))
    return math.degrees(th1), math.degrees(th2)  # (hip, knee)

def set_all_minmax(kit):
    for ch in range(8):
        kit.servo[ch].set_pulse_width_range(MIN_US, MAX_US)

def apply_angles(kit, angles_deg):
    for ch in range(8):
        kit.servo[ch].angle = clamp(angles_deg[ch])

def stand_pose_from_ik():
    th1_deg, th2_deg = ik_under_hip(H)
    # map IK -> servo cho 4 chân
    out = [0.0]*8
    for i, (h, k) in enumerate(zip(HIP_CH, KNEE_CH)):
        out[h] = clamp(HIP_OFF[i]  + HIP_SIGN[i]  * th1_deg)
        out[k] = clamp(KNEE_OFF[i] + KNEE_SIGN[i] * th2_deg)
    return out

def getch_nowait():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

def main():
    kit = ServoKit(channels=16)
    set_all_minmax(kit)

    # Tư thế bắt đầu
    apply_angles(kit, INIT_ANGLES)
    print("Start pose applied. Phím: w=Đứng, s=Bắt đầu, q=Thoát")

    # Bật chế độ đọc phím không cần Enter
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            c = getch_nowait()
            if c == 'w':
                angles = stand_pose_from_ik()
                apply_angles(kit, angles)
                print("→ Stand pose:", [round(a,1) for a in angles])
            elif c == 's':
                apply_angles(kit, INIT_ANGLES)
                print("→ Back to start pose.")
            elif c == 'q':
                print("Bye.")
                break
            time.sleep(0.01)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

if __name__ == "__main__":
    main()
