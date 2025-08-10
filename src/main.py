import math, time
from adafruit_servokit import ServoKit

# ===== PCA9685 channels =====
KNEE_CH = 0   # Servo 1 (cẳng)
HIP_CH  = 1   # Servo 2 (đùi)

kit = ServoKit(channels=16)
for ch in [KNEE_CH, HIP_CH]:
    kit.servo[ch].actuation_range = 180
    kit.servo[ch].set_pulse_width_range(500, 2500)

# ===== Hình học =====
L1 = 100.0   # đùi (mm)
L2 = 100.0   # cẳng (mm)

# ===== Hiệu chuẩn (chỉnh theo thực tế) =====
OFFSET = {HIP_CH: 90.0, KNEE_CH: 90.0}
SIGN   = {HIP_CH: +1.0, KNEE_CH: +1.0}   # nếu quay ngược mũi tên -> đổi dấu khớp đó

def clamp(x, lo, hi): 
    return max(lo, min(hi, x))

def ik_2link(x, z):
    """Trả về (θ_hip, θ_knee) rad; nghiệm ưu tiên gần duỗi thẳng (knee nhỏ)."""
    r2 = x*x + z*z
    r  = math.sqrt(r2)
    if not (abs(L1 - L2) <= r <= (L1 + L2)):
        raise ValueError(f"Out of reach: r={r:.1f} mm")
    c2 = clamp((r2 - L1*L1 - L2*L2) / (2*L1*L2), -1.0, 1.0)
    th2 = math.acos(c2)  # 0 = duỗi thẳng, -> pi = gập mạnh
    th1 = math.atan2(z, x) - math.atan2(L2*math.sin(th2), L1 + L2*math.cos(th2))
    return th1, th2

def to_servo_deg(rad, ch):
    return OFFSET[ch] + SIGN[ch] * math.degrees(rad)

def set_foot(x_mm, z_mm, settle=0.4):
    th1, th2 = ik_2link(x_mm, z_mm)
    hip_deg  = to_servo_deg(th1, HIP_CH)
    knee_deg = to_servo_deg(th2, KNEE_CH)
    kit.servo[HIP_CH].angle  = clamp(hip_deg,  0, 180)
    kit.servo[KNEE_CH].angle = clamp(knee_deg, 0, 180)
    time.sleep(settle)
    print(f"LIE pose -> HIP={hip_deg:.1f}°, KNEE={knee_deg:.1f}°  (x={x_mm:.1f}mm, z={z_mm:.1f}mm)")

def lie_pose(height_mm=70.0, epsilon_mm=4.0):
    """Đặt tư thế nằm: duỗi gần thẳng với chiều cao thân 'height_mm'."""
    r = L1 + L2 - epsilon_mm
    if height_mm > r:
        raise ValueError("height_mm quá lớn so với tầm với khi duỗi thẳng.")
    x = math.sqrt(r*r - height_mm*height_mm)
    z = -height_mm
    return set_foot(x, z)

if __name__ == "__main__":
    # Ví dụ đúng theo hình: h=70mm, duỗi gần thẳng (ε=4mm)
    lie_pose(height_mm=70.0, epsilon_mm=4.0)
