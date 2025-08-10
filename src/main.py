import math
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

KNEE_CH = 0   # servo 1 (kênh 0)
HIP_CH  = 1   # servo 2 (kênh 1)

# PWM range (chỉnh theo servo của bạn)
for ch in (KNEE_CH, HIP_CH):
    kit.servo[ch].actuation_range = 180
    kit.servo[ch].set_pulse_width_range(500, 2500)

# ---- Mapping nhanh cho kênh 0 (bạn chỉnh 2 line này nếu lệch) ----
K0_SIGN   = +1    # nếu quay ngược chiều mong muốn, đổi thành -1
K0_OFFSET = 90.0  # nếu zero lệch, tăng/giảm offset này

def clamp(a, lo=0.0, hi=180.0): 
    return max(lo, min(hi, a))

def knee_servo_from_alpha(alpha_deg):
    """
    alpha_deg: góc 'đường đỏ' so với phương thẳng đứng đi xuống (độ).
               Ví dụ 0: thẳng đứng, 30: nghiêng về trước 30°.
    Trả về góc servo cho kênh 0.
    """
    theta_knee_deg = -180.0 + 2.0 * alpha_deg   # IK: θ2 = -180 + 2α
    servo_deg = K0_OFFSET + K0_SIGN * theta_knee_deg
    return clamp(servo_deg)

def sit_left_leg(alpha_deg=30.0):
    """Đặt tư thế ngồi: hip (kênh 1) = 0°, knee (kênh 0) theo đường đỏ."""
    kit.servo[HIP_CH].angle  = clamp(0.0)                 # giữ 0°
    kit.servo[KNEE_CH].angle = knee_servo_from_alpha(alpha_deg)
    print(f"Sit: alpha={alpha_deg:.1f}°  -> knee_servo={kit.servo[KNEE_CH].angle:.1f}°, hip=0°")

if __name__ == "__main__":
    # Ví dụ: 'đường đỏ' nghiêng 30° so với phương thẳng đứng
    sit_left_leg(alpha_deg=60.0)
