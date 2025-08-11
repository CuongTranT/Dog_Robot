# dog_start_and_stand_xyconst.py
# - Khởi động: set INIT_ANGLES
# - Nhấn 'w': dùng STAND_X, STAND_Y -> IK -> set kênh 0,1
# - Nhấn 's': về tư thế bắt đầu; 'q': thoát

import sys, termios, tty, select, math, time
from adafruit_servokit import ServoKit

# ---------- CONFIG ----------
INIT_ANGLES = [100, 180, 110, 180, 70, 20, 85, 20]
L1 = 10.0     # cm
L2 = 10.0     # cm
STAND_X = 0.0 # cm  (bạn sửa ở đây)
STAND_Y = 18.0# cm  (bạn sửa ở đây)
ELBOW_DOWN = True  # nếu chiều ngược, thử đổi False

MIN_US, MAX_US = 600, 2400
HIP_CH, KNEE_CH = 0, 1  # chỉ thử 2 kênh này

# ---------- IK & helpers ----------
def clamp(v, lo, hi): return max(lo, min(hi, v))

def ik_2R_xy(x, y, elbow_down=True):
    # Quy ước: gốc tại hông; x sang phải; y dương xuống
    r2 = x*x + y*y
    if r2 < (L1-L2)**2 - 1e-9 or r2 > (L1+L2)**2 + 1e-9:
        raise ValueError(f"({x:.2f},{y:.2f}) ngoài tầm với")

    c2 = (r2 - L1*L1 - L2*L2) / (2*L1*L2)
    c2 = clamp(c2, -1.0, 1.0)
    s2 = math.sqrt(max(0.0, 1.0 - c2*c2))
    if not elbow_down:
        s2 = -s2

    q2 = math.atan2(s2, c2)
    a1 = math.atan2(y, x) - math.atan2(L2*s2, L1 + L2*c2)
    a2 = a1 + q2
    return math.degrees(a1), math.degrees(a2)

def move_servo_smooth(kit, ch, target_deg, step=2.0, dt=0.01):
    cur = kit.servo[ch].angle
    tgt = clamp(target_deg, 0, 180)
    if cur is None:
        kit.servo[ch].angle = tgt
        return
    sgn = 1.0 if tgt >= cur else -1.0
    a = cur
    while (a - tgt) * sgn < 0:
        a += sgn * step
        if (a - tgt) * sgn > 0:
            a = tgt
        kit.servo[ch].angle = clamp(a, 0, 180)
        time.sleep(dt)

def apply_init(kit):
    for ch, ang in enumerate(INIT_ANGLES):
        kit.servo[ch].set_pulse_width_range(MIN_US, MAX_US)
        kit.servo[ch].angle = clamp(ang, 0, 180)

# ---------- non-blocking key read ----------
class KB:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self
    def __exit__(self, *_):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
    def readch(self, timeout=0.05):
        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1)
        return ''

# ---------- Main ----------
def main():
    kit = ServoKit(channels=16)
    apply_init(kit)
    print("Sẵn sàng. Nhấn 'w' để đứng theo STAND_X/STAND_Y; 's' về INIT; 'q' thoát.")
    print(f"STAND_X={STAND_X} cm, STAND_Y={STAND_Y} cm")

    with KB() as kb:
        while True:
            c = kb.readch()
            if c == 'q':
                print("Thoát.")
                break
            elif c == 's':
                print("Về INIT.")
                apply_init(kit)
            elif c == 'w':
                try:
                    a1, a2 = ik_2R_xy(STAND_X, STAND_Y, ELBOW_DOWN)
                except ValueError as e:
                    print("IK lỗi:", e); continue
                print(f"IK -> alpha1={a1:.2f}°, alpha2={a2:.2f}°")
                move_servo_smooth(kit, HIP_CH,  a1)
                move_servo_smooth(kit, KNEE_CH, a2)

if __name__ == "__main__":
    main()
