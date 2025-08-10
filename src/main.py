import math, time
from adafruit_servokit import ServoKit

# ===== PCA9685 channels =====
KNEE_CHS = [0, 2]   # servo 1 (cẳng) -> kênh 0 và 2
HIP_CHS  = [1, 3]   # servo 2 (đùi)  -> kênh 1 và 3

kit = ServoKit(channels=16)
for ch in KNEE_CHS + HIP_CHS:
    kit.servo[ch].actuation_range = 180
    kit.servo[ch].set_pulse_width_range(500, 2500)

# ===== Hình học =====
L1 = 100.0  # đùi (mm)
L2 = 100.0  # cẳng (mm)

# ===== Hiệu chuẩn (mặc định giống nhau; có thể chỉnh riêng từng kênh) =====
OFFSET = {0: 90.0, 1: 90.0, 2: 90.0, 3: 90.0}
SIGN   = {0: +1.0, 1: +1.0, 2: +1.0, 3: +1.0}  # nếu quay ngược mũi tên -> đổi dấu kênh đó

def clamp(x, lo, hi): 
    return max(lo, min(hi, x))

def ik_2link(x, z):
    """(θ_hip, θ_knee) rad; nghiệm gần duỗi thẳng."""
    r2 = x*x + z*z
    r  = math.sqrt(r2)
    if not (abs(L1 - L2) <= r <= (L1 + L2)):
        raise ValueError(f"Out of reach: r={r:.1f} mm")
    c2 = clamp((r2 - L1*L1 - L2*L2) / (2*L1*L2), -1.0, 1.0)
    th2 = math.acos(c2)  # 0=duỗi, tăng lên=gập
    th1 = math.atan2(z, x) - math.atan2(L2*math.sin(th2), L1 + L2*math.cos(th2))
    return th1, th2

def set_foot_all(x_mm, z_mm, settle=0.4):
    th1, th2 = ik_2link(x_mm, z_mm)
    hip_deg  = math.degrees(th1)
    knee_deg = math.degrees(th2)

    # ghi cho cả nhóm kênh
    for ch in HIP_CHS:
        val = clamp(OFFSET[ch] + SIGN[ch]*hip_deg, 0, 180)
        kit.servo[ch].angle = val
    for ch in KNEE_CHS:
        val = clamp(OFFSET[ch] + SIGN[ch]*knee_deg, 0, 180)
        kit.servo[ch].angle = val

    time.sleep(settle)
    print(f"LIE pose -> HIP={hip_deg:.1f}°, KNEE={knee_deg:.1f}°  (x={x_mm:.1f}mm, z={z_mm:.1f}mm)")

def lie_pose(height_mm=70.0, epsilon_mm=4.0):
    """Đặt tư thế nằm cho 4 kênh: 0~3. 2=0, 3=1."""
    r = L1 + L2 - epsilon_mm
    if height_mm > r:
        raise ValueError("height_mm quá lớn so với tầm với khi duỗi thẳng.")
    x = math.sqrt(r*r - height_mm*height_mm)
    z = -height_mm
    set_foot_all(x, z)

if __name__ == "__main__":
    # Ví dụ: h=70mm, duỗi gần thẳng (ε=4mm)
    lie_pose(height_mm=70.0, epsilon_mm=4.0)
