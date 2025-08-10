import math, time, sys, termios, tty, select
from adafruit_servokit import ServoKit

# ===== Kênh =====
RIGHT_KNEE = [0, 2]   # servo 1 (cẳng) bên phải
RIGHT_HIP  = [1, 3]   # servo 2 (đùi)  bên phải
LEFT_KNEE  = [4, 6]   # trái = copy từ phải
LEFT_HIP   = [5, 7]

# Cặp mirror: kênh trái -> kênh phải
PAIR = {4:0, 6:2, 5:1, 7:3}

ALL_CH = RIGHT_KNEE + RIGHT_HIP + LEFT_KNEE + LEFT_HIP

# ===== Servo =====
kit = ServoKit(channels=16)
for ch in ALL_CH:
    kit.servo[ch].actuation_range = 180
    kit.servo[ch].set_pulse_width_range(500, 2500)

# ===== Hình học =====
L1 = 100.0
L2 = 100.0

# ===== Hiệu chuẩn CHỈ CHO BÊN PHẢI (trái sẽ copy y nguyên kết quả phải) =====
OFFSET = {0:90.0, 1:90.0, 2:90.0, 3:90.0}   # chỉ dùng 0..3
SIGN   = {0:+1.0, 1:+1.0, 2:+1.0, 3:+1.0}   # kênh phải nào quay ngược -> đổi dấu

# ===== Pose (x,z) =====
STAND_X, STAND_Z = 50.0, -160.0
SIT_X,   SIT_Z   = 100,  -100.0

cur_x, cur_z = None, None

def clamp(x, lo, hi): return max(lo, min(hi, x))

def ik_2link(x, z):
    r2 = x*x + z*z
    r  = math.sqrt(r2)
    if not (abs(L1-L2) <= r <= (L1+L2)):
        raise ValueError(f"Out of reach: r={r:.1f} mm")
    c2 = clamp((r2 - L1*L1 - L2*L2) / (2*L1*L2), -1.0, 1.0)
    th2 = math.acos(c2)  # knee: 0=duỗi thẳng
    th1 = math.atan2(z, x) - math.atan2(L2*math.sin(th2), L1 + L2*math.cos(th2))
    return th1, th2  # hip, knee (rad)

def set_right_and_copy_left(hip_joint_deg, knee_joint_deg):
    """Áp OFFSET/SIGN cho bên phải -> ra góc servo phải; trái = copy đúng kênh phải tương ứng."""
    right_servo = {}
    # Ghi HIP phải (kênh 1,3)
    for ch in RIGHT_HIP:
        val = clamp(OFFSET[ch] + SIGN[ch]*hip_joint_deg, 0, 180)
        kit.servo[ch].angle = val
        right_servo[ch] = val
    # Ghi KNEE phải (kênh 0,2)
    for ch in RIGHT_KNEE:
        val = clamp(OFFSET[ch] + SIGN[ch]*knee_joint_deg, 0, 180)
        kit.servo[ch].angle = val
        right_servo[ch] = val
    # Copy sang trái (KHÔNG offset/sign, KHÔNG 180-)
    for left_ch, right_ch in PAIR.items():
        kit.servo[left_ch].angle = right_servo[right_ch]

def set_pose_all(x_mm, z_mm, steps=30, dt=0.02):
    global cur_x, cur_z
    if cur_x is None or cur_z is None:
        cur_x, cur_z = x_mm, z_mm
    for i in range(1, steps+1):
        xi = cur_x + (x_mm - cur_x)*i/steps
        zi = cur_z + (z_mm - cur_z)*i/steps
        th1, th2 = ik_2link(xi, zi)
        hip_deg  = math.degrees(th1)
        knee_deg = math.degrees(th2)
        set_right_and_copy_left(hip_deg, knee_deg)
        time.sleep(dt)
    cur_x, cur_z = x_mm, z_mm

def pose_stand(): set_pose_all(STAND_X, STAND_Z)
def pose_sit():   set_pose_all(SIT_X,   SIT_Z)

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
                pose_stand(); print("-> Stand")
            elif c.lower() == 's':
                pose_sit();   print("-> Sit")
            elif c.lower() == 'q':
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
