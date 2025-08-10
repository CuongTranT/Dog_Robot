import math, time
from adafruit_servokit import ServoKit

# ===== Nhóm kênh =====
RIGHT_KNEE = [0, 2]      # servo 1 (cẳng) bên phải
RIGHT_HIP  = [1, 3]      # servo 2 (đùi)  bên phải
LEFT_KNEE  = [4, 6]      # servo 1 (cẳng) bên trái  (== phải)
LEFT_HIP   = [5, 7]      # servo 2 (đùi)  bên trái  (== phải)

# Map: kênh trái -> kênh phải tương ứng
PAIR = {4:0, 6:2, 5:1, 7:3}

kit = ServoKit(channels=16)
for ch in RIGHT_KNEE + RIGHT_HIP + LEFT_KNEE + LEFT_HIP:
    kit.servo[ch].actuation_range = 180
    kit.servo[ch].set_pulse_width_range(500, 2500)

# ===== Hình học =====
L1 = 100.0
L2 = 100.0

# ===== Offset & chiều quay (áp cho kênh PHẢI; trái sẽ copy y nguyên) =====
OFFSET = {0:90.0, 1:90.0, 2:90.0, 3:90.0}
SIGN   = {0:+1.0, 1:+1.0, 2:+1.0, 3:+1.0}  # nếu kênh nào quay ngược -> đổi dấu kênh đó

def clamp(x, lo, hi): return max(lo, min(hi, x))

def ik_2link(x, z):
    r2 = x*x + z*z
    r  = math.sqrt(r2)
    if not (abs(L1-L2) <= r <= (L1+L2)):
        raise ValueError(f"Out of reach: r={r:.1f} mm")
    c2 = clamp((r2 - L1*L1 - L2*L2) / (2*L1*L2), -1.0, 1.0)
    th2 = math.acos(c2)  # 0=duỗi, tăng lên=gập
    th1 = math.atan2(z, x) - math.atan2(L2*math.sin(th2), L1 + L2*math.cos(th2))
    return th1, th2

def lie_pose(height_mm=70.0, epsilon_mm=4.0, settle=0.4):
    # Tính (x,z) cho tư thế nằm (gần duỗi thẳng)
    r = L1 + L2 - epsilon_mm
    if height_mm > r:
        raise ValueError("height_mm quá lớn so với tầm với khi duỗi thẳng.")
    x = math.sqrt(r*r - height_mm*height_mm)
    z = -height_mm

    # IK → góc khớp (rad)
    th1, th2 = ik_2link(x, z)
    hip_joint_deg  = math.degrees(th1)
    knee_joint_deg = math.degrees(th2)

    # Ghi PHẢI (có OFFSET/SIGN riêng)
    right_angles = {}
    for ch in RIGHT_HIP:
        a = clamp(OFFSET[ch] + SIGN[ch]*hip_joint_deg, 0, 180)
        right_angles[ch] = a
        kit.servo[ch].angle = a
    for ch in RIGHT_KNEE:
        a = clamp(OFFSET[ch] + SIGN[ch]*knee_joint_deg, 0, 180)
        right_angles[ch] = a
        kit.servo[ch].angle = a

    # ===== MIRROR MỚI: TRÁI = PHẢI (KHÔNG 180 - GÓC) =====
    for ch_left, ch_right in PAIR.items():
        kit.servo[ch_left].angle = right_angles[ch_right]

    time.sleep(settle)
    print("Lie pose set. Copied right → left (no 180-).")

if __name__ == "__main__":
    lie_pose(height_mm=70.0, epsilon_mm=4.0)
