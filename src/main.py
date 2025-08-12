import math
import time
from adafruit_servokit import ServoKit

# ==== CONFIG ====
L1 = 10.0  # Chiều dài khâu 1 (hip)
L2 = 10.0  # Chiều dài khâu 2 (knee)
MIN_US, MAX_US = 600, 2400

# Tọa độ mục tiêu cho mỗi chân
# A(trái-trên), B(phải-trên), C(trái-dưới), D(phải-dưới)
SITTING_POINTS = {
    "A": (0.0, 9.0),   # kênh 0,1 - tư thế ngồi
    "B": (0.0, 9.0),   # kênh 2,3 - tư thế ngồi
    "C": (0.0, 9.0),   # kênh 4,5 - tư thế ngồi
    "D": (0.0, 9.0),   # kênh 6,7 - tư thế ngồi
}

STANDING_POINTS = {
    "A": (0.0, 18.0),  # kênh 0,1 - tư thế đứng
    "B": (0.0, 18.0),  # kênh 2,3 - tư thế đứng
    "C": (0.0, 18.0),  # kênh 4,5 - tư thế đứng
    "D": (0.0, 18.0),  # kênh 6,7 - tư thế đứng
}

# Kênh servo tương ứng từng chân
SERVO_CHANNELS = {
    "A": {"hip": 0, "knee": 1},  # A: kênh 0,1
    "B": {"hip": 2, "knee": 3},  # B: kênh 2,3
    "C": {"hip": 4, "knee": 5},  # C: kênh 4,5
    "D": {"hip": 6, "knee": 7},  # D: kênh 6,7
}

# Hiệu chỉnh offset từng kênh (độ) để căn tư thế đứng chuẩn
SERVO_OFFSETS = {
    0: 0,  # CH0  hip A
    1: 0,  # CH1  knee A
    2: 0,  # CH2  hip B
    3: 0,  # CH3  knee B
    4: 0,  # CH4  hip C
    5: 0,  # CH5  knee C
    6: 0,  # CH6  hip D
    7: 0,  # CH7  knee D
}

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def ik_2d(x, y, elbow_down=True):
    """Tính động học ngược cho tay máy 2 khâu"""
    r2 = x*x + y*y
    if r2 < (L1 - L2)**2 - 1e-9 or r2 > (L1 + L2)**2 + 1e-9:
        raise ValueError("Điểm đặt ngoài tầm với")

    c2 = (r2 - L1*L1 - L2*L2) / (2 * L1 * L2)
    c2 = clamp(c2, -1.0, 1.0)
    s2 = math.sqrt(1.0 - c2*c2)
    if not elbow_down:
        s2 = -s2

    q2 = math.atan2(s2, c2)
    alpha1 = math.atan2(y, x) - math.atan2(L2 * s2, L1 + L2 * c2)  # khâu 1
    alpha0 = alpha1 + q2                                           # khâu 2
    return math.degrees(alpha0), math.degrees(alpha1)

def get_servo_angle(channel, alpha0, alpha1):
    """Tính góc servo dựa trên kênh và góc alpha"""
    if channel % 2 == 0:  # Hip servo (kênh chẵn)
        # Servo dưới hướng sang phải
        return 90 + alpha0 + SERVO_OFFSETS[channel]
    else:  # Knee servo (kênh lẻ)
        # Servo trên hướng sang trái (ngược chiều)
        return 90 - alpha1 + SERVO_OFFSETS[channel]

def calculate_init_angles():
    """Tính toán góc khởi tạo cho tư thế ngồi trung tính"""
    init_angles = [0] * 8
    
    # Tính góc cho tọa độ ngồi (0.0, 9.0) cm
    x, y = 0.0, 9.0
    alpha0, alpha1 = ik_2d(x, y, elbow_down=True)
    
    print(f"=== TÍNH TOÁN GÓC KHỞI TẠO ===")
    print(f"Tọa độ ngồi: ({x}, {y}) cm")
    print(f"Góc alpha0 = {alpha0:.2f}°, alpha1 = {alpha1:.2f}°")
    print()
    
    # Tính góc servo cho từng kênh
    for ch in range(8):
        angle = get_servo_angle(ch, alpha0, alpha1)
        init_angles[ch] = clamp(angle, 0, 180)
        print(f"CH{ch}: {angle:.2f}° → clamp = {init_angles[ch]:.2f}°")
    
    print()
    return init_angles

# Góc khởi tạo cho tất cả các kênh (tư thế ngồi trung tính)
INIT_ANGLES = calculate_init_angles()

def move_leg(kit, leg_name, x, y, elbow_down=True):
    alpha0, alpha1 = ik_2d(x, y, elbow_down)
    hip_ch  = SERVO_CHANNELS[leg_name]["hip"]
    knee_ch = SERVO_CHANNELS[leg_name]["knee"]

    angle_hip  = clamp(get_servo_angle(hip_ch, alpha0, alpha1), 0, 180)
    angle_knee = clamp(get_servo_angle(knee_ch, alpha0, alpha1), 0, 180)

    kit.servo[hip_ch].angle  = angle_hip
    kit.servo[knee_ch].angle = angle_knee

    print(f"[{leg_name}] G=({x}, {y}) cm")
    print(f"  α0 = {alpha0:.2f}°, α1 = {alpha1:.2f}°")
    print(f"  Servo CH{hip_ch} = {angle_hip:.2f}°, CH{knee_ch} = {angle_knee:.2f}°")

def main():
    kit = ServoKit(channels=16)

    print("=== KHỞI TẠO ROBOT CHÓ ===")
    print(f"Góc khởi tạo: {INIT_ANGLES}")
    print()

    # Cấu hình và đặt tư thế ban đầu (ngồi)
    for ch in range(8):
        kit.servo[ch].set_pulse_width_range(MIN_US, MAX_US)
        kit.servo[ch].angle = clamp(INIT_ANGLES[ch], 0, 180)
    time.sleep(0.5)

    print("=== CHUYỂN SANG TƯ THẾ ĐỨNG ===")
    # Điều khiển từng chân về vị trí đứng
    for leg, (x, y) in STANDING_POINTS.items():
        move_leg(kit, leg, x, y, elbow_down=True)
    
    print("\n=== DI CHUYỂN KÊNH 0 VÀ 1 VỀ 180° ===")
    # Di chuyển kênh 0 và 1 về vị trí 180 độ
    kit.servo[0].angle = 180
    kit.servo[1].angle = 180
    print("CH0: 180°")
    print("CH1: 180°")
    print("Kênh 0 và 1 đã được di chuyển về vị trí 180°")

if __name__ == "__main__":
    main()
