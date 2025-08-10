import math, time, sys, termios, tty, select
from adafruit_servokit import ServoKit

# ===== Kênh =====
RIGHT_KNEE = [0, 2]
RIGHT_HIP  = [1, 3]
LEFT_KNEE  = [4, 6]
LEFT_HIP   = [5, 7]
ALL_CH = RIGHT_KNEE + RIGHT_HIP + LEFT_KNEE + LEFT_HIP

# ===== Servo =====
kit = ServoKit(channels=16)
for ch in ALL_CH:
    kit.servo[ch].actuation_range = 180
    kit.servo[ch].set_pulse_width_range(500, 2500)

# ===== Hình học =====
L1 = 100.0
L2 = 100.0

# ===== Hiệu chuẩn (chỉnh theo thực tế của bạn) =====
OFFSET = {ch: 90.0 for ch in ALL_CH}
SIGN   = {ch: +1.0 for ch in ALL_CH}   # kênh nào quay ngược mũi tên -> đặt -1

# ===== Pose định nghĩa bằng (x,z) =====
STAND_X, STAND_Z = 50.0, -160.0
SIT_X,   SIT_Z   = 40.0,  -80.0

# trạng thái hiện tại để nội suy mượt
cur_x, cur_z = None, None

def clamp(x, lo, hi): return max(lo, min(hi, x))

def ik_2link(x, z):
    r2 = x*x + z*z
    r  = math.sqrt(r2)
    if not (abs(L1-L2) <= r <= (L1+L2)):
        raise ValueError(f"Out of reach: r={r:.1f} mm")
    c2 = clamp((r2 - L1*L1 - L2*L2) / (2*L1*L2), -1.0, 1.0)
    th2 = math.acos(c2)  # 0=duỗi thẳng
    th1 = math.atan2(z, x) - math.atan2(L2*math.sin(th2), L1 + L2*math.cos(th2))
    return th1, th2  # hip, knee (rad)

def write_leg_group(hip_deg, knee_deg, hips, knees):
    for ch in hips:
        kit.servo[ch].angle = clamp(OFFSET[ch] + SIGN[ch]*hip_deg, 0, 180)
    for ch in knees:
        kit.servo[ch].angle = clamp(OFFSET[ch] + SIGN[ch]*knee_deg, 0, 180)

def set_pose_all(x_mm, z_mm, steps=30, dt=0.02):
    global cur_x, cur_z
    if cur_x is None or cur_z is None:
        cur_x, cur_z = x_mm, z_mm  # lần đầu đặt thẳng

    for i in range(1, steps+1):
        xi = cur_x + (x_mm - cur_x)*i/steps
        zi = cur_z + (z_mm - cur_z)*i/steps
        th1, th2 = ik_2link(xi, zi)
        hip_deg  = math.degrees(th1)
        knee_deg = math.degrees(th2)
        # PHẢI
        write_leg_group(hip_deg, knee_deg, RIGHT_HIP, RIGHT_KNEE)
        # TRÁI = copy PHẢI (không 180 - góc)
        write_leg_group(hip_deg, knee_deg, LEFT_HIP, LEFT_KNEE)
        time.sleep(dt)

    cur_x, cur_z = x_mm, z_mm

def pose_stand():
    set_pose_all(STAND_X, STAND_Z)

def pose_sit():
    set_pose_all(SIT_X, SIT_Z)

# ===== Bàn phím: w = đứng, s = ngồi, q = thoát =====
def kbhit(timeout=0.1):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if dr else None

if __name__ == "__main__":
    print("Controls:  w=Stand,  s=Sit,  q=Quit")
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        pose_sit()  # khởi động: ngồi
        while True:
            c = kbhit(0.1)
            if not c: 
                continue
            if c.lower() == 'w':
                pose_stand()
                print("-> Stand")
            elif c.lower() == 's':
                pose_sit()
                print("-> Sit")
            elif c.lower() == 'q':
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
