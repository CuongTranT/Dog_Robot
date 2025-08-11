# dog_start_and_stand_xyconst_v2.py
# - Khởi động: set INIT_ANGLES
# - Nhấn 'w': dùng STAND_X, STAND_Y -> IK -> set kênh 0 (hip=S2), 1 (knee=S1)
#   KNEE_CMD = alpha1 + 90° theo yêu cầu mới
# - Nhấn 's': về INIT; 'q': thoát

import sys, termios, tty, select, math, time
from adafruit_servokit import ServoKit

# ---------- CONFIG ----------
INIT_ANGLES = [100, 180, 110, 180, 70, 20, 85, 20]
L1 = 10.0
L2 = 10.0
STAND_X = 0.0   # cm  (sửa tại đây)
STAND_Y = 18.0  # cm  (sửa tại đây)
ELBOW_DOWN = True

KNEE_ALPHA1_OFFSET = +90.0  # α1 + 90° (yêu cầu của bạn)

MIN_US, MAX_US = 600, 2400
HIP_CH, KNEE_CH = 0, 1      # chỉ thử 2 kênh này

# ---------- IK & helpers ----------
def clamp(v, lo, hi): return max(lo, min(hi, v))

def ik_global_angles(x, y, elbow_down=True):
    """
    IK 2R với quy ước: gốc tại hông; x sang phải; y dương xuống.
    Trả về (alpha1_deg, alpha2_deg) theo ký hiệu mới của bạn:
      - alpha2: góc khâu 1 (hip) so với +x
      - alpha1: góc khâu 2 (knee) so với +x
    """
    r2 = x*x + y*y
    if r2 < (L1-L2)**2 - 1e-9 or r2 > (L1+L2)**2 + 1e-9:
        raise ValueError(f"({x:.2f},{y:.2f}) ngoài tầm với của 2 khâu")

    c2 = (r2 - L1*L1 - L2*L2) / (2*L1*L2)
    c2 = clamp(c2, -1.0, 1.0)
    s2 = math.sqrt(max(0.0, 1.0 - c2*c2))
    if not elbow_down: s2 = -s2

    q2 = math.atan2(s2, c2)
    # th1: hướng khâu 1; th2: hướng khâu 2 (toàn cục)
    th1 = math.atan2(y, x) - math.atan2(L2*s2, L1 + L2*c2)  # = α2 (mới)
    th2 = th1 + q2                                          # = α1 (mới)

    alpha2_deg = math.degrees(th1)  # hip (S2)
    alpha1_deg = math.degrees(th2)  # knee (S1)
    return alpha1_deg, alpha2_deg

def move_servo_smooth(kit, ch, target_deg, step=2.0, dt=0.01):
    cur = kit.servo[ch].angle
    tgt = clamp(target_deg, 0, 180)
    if cur is None:
        kit.servo[ch].angle = tgt; return
    sgn = 1.0 if tgt >= cur else -1.0
    a = cur
    while (a - tgt) * sgn < 0:
        a += sgn * step
        if (a - tgt) * sgn > 0: a = tgt
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
        tty.setcbreak(self.fd); return self
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
    print(f"Sẵn sàng. 'w' -> (x,y)=({STAND_X},{STAND_Y}) cm; 's' về INIT; 'q' thoát.")

    with KB() as kb:
        while True:
            c = kb.readch()
            if c == 'q': break
            elif c == 's':
                apply_init(kit); print("INIT")
            elif c == 'w':
                try:
                    alpha1, alpha2 = ik_global_angles(STAND_X, STAND_Y, ELBOW_DOWN)
                except ValueError as e:
                    print("IK lỗi:", e); continue

                hip_cmd  = alpha2
                knee_cmd = alpha1 + KNEE_ALPHA1_OFFSET  # α1 + 90°

                # cảnh báo biên
                if not (0 <= hip_cmd <= 180) or not (0 <= knee_cmd <= 180):
                    print(f"Cảnh báo: hip={hip_cmd:.2f}°, knee={knee_cmd:.2f}° vượt biên -> sẽ clamp")

                print(f"α2(hip)={alpha2:.2f}°, α1(knee)={alpha1:.2f}° -> "
                      f"S2={hip_cmd:.2f}°, S1={knee_cmd:.2f}°")

                move_servo_smooth(kit, HIP_CH,  hip_cmd)
                move_servo_smooth(kit, KNEE_CH, knee_cmd)

if __name__ == "__main__":
    main()
