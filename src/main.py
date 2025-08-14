import math
import time
from Adafruit_PCA9685 import PCA9685

# ==============================
# ⚙️ CẤU HÌNH PCA9685
# ==============================
pwm = PCA9685(busnum=1)
pwm.set_pwm_freq(60)  # MG996R hoạt động tốt ở 60Hz

# ==============================
# 📐 Chuyển góc sang giá trị PWM (12-bit)
# ==============================
def angle_to_pwm(angle_deg):
    pulse_min = 500    # us (0°)
    pulse_max = 2500   # us (180°)
    pulse_us = pulse_min + (pulse_max - pulse_min) * angle_deg / 180
    pulse_len = 1000000.0 / 60 / 4096  # tick time @60Hz
    pwm_val = int(pulse_us / pulse_len)
    return pwm_val

# ==============================
# 🚦 Điều khiển servo
# ==============================
def set_servo_angle(channel, angle_deg):
    angle_deg = max(0, min(180, angle_deg))
    pwm_val = angle_to_pwm(angle_deg)
    pwm.set_pwm(channel, 0, pwm_val)

# ==============================
# 🤖 Hệ IK 2 bậc chính xác: tính α1 và α2
# ==============================
def inverse_kinematics(x, y, L1=10.0, L2=10.0):
    """
    Tính toán inverse kinematics cho chân robot 2 bậc tự do
    L1: chiều dài đùi (femur)
    L2: chiều dài ống chân (tibia)
    x, y: tọa độ điểm đích trong hệ tọa độ chân
    """
    # Tính khoảng cách từ gốc đến điểm đích
    D = math.sqrt(x**2 + y**2)
    
    # Kiểm tra điểm có trong tầm với không
    if D > (L1 + L2) or D < abs(L1 - L2):
        raise ValueError(f"Điểm ({x}, {y}) ngoài tầm với. D={D:.2f}, L1+L2={L1+L2}")
    
    # Tính góc alpha2 (góc giữa L1 và L2) sử dụng định lý cosin
    cos_alpha2 = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    cos_alpha2 = max(-1, min(1, cos_alpha2))  # Giới hạn trong [-1, 1] để tránh lỗi
    alpha2 = math.acos(cos_alpha2)
    
    # Tính góc beta (góc giữa L1 và đường thẳng đến điểm đích)
    cos_beta = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    cos_beta = max(-1, min(1, cos_beta))
    beta = math.acos(cos_beta)
    
    # Tính góc alpha1 (góc giữa L1 và trục Y)
    # Sử dụng atan2 để xác định góc chính xác
    theta = math.atan2(y, x)
    alpha1 = theta + beta
    
    # Chuyển về độ
    alpha1_deg = math.degrees(alpha1)
    alpha2_deg = math.degrees(alpha2)
    
    return alpha1_deg, alpha2_deg

# ==============================
# 🦿 Điều khiển chân robot với IK chính xác
# ==============================
def move_leg(leg_id, x, y):
    """
    Di chuyển chân robot đến vị trí (x, y)
    leg_id: 0=chân trước phải, 1=chân trước trái, 2=chân sau phải, 3=chân sau trái
    """
    try:
        alpha1_deg, alpha2_deg = inverse_kinematics(x, y)
        
        print(f"Chân {leg_id}: ({x}, {y}) → α1 = {alpha1_deg:.2f}°, α2 = {alpha2_deg:.2f}°")
        
        # Điều chỉnh góc cho từng chân dựa trên hướng lắp đặt
        if leg_id == 0:  # Chân trước phải
            hip_angle = 90 + alpha1_deg
            knee_angle = 90 + alpha2_deg
            set_servo_angle(0, hip_angle)   # Hip servo
            set_servo_angle(1, knee_angle)  # Knee servo
            
        elif leg_id == 1:  # Chân trước trái
            hip_angle = 90 - alpha1_deg
            knee_angle = 90 - alpha2_deg
            set_servo_angle(2, hip_angle)   # Hip servo
            set_servo_angle(3, knee_angle)  # Knee servo
            
        elif leg_id == 2:  # Chân sau phải
            hip_angle = 90 + alpha1_deg
            knee_angle = 90 + alpha2_deg
            set_servo_angle(4, hip_angle)   # Hip servo
            set_servo_angle(5, knee_angle)  # Knee servo
            
        elif leg_id == 3:  # Chân sau trái
            hip_angle = 90 - alpha1_deg
            knee_angle = 90 - alpha2_deg
            set_servo_angle(6, hip_angle)   # Hip servo
            set_servo_angle(7, knee_angle)  # Knee servo
            
    except ValueError as e:
        print(f"Lỗi chân {leg_id}: {e}")

# ==============================
# 🚶‍♂️ Điều khiển toàn bộ robot
# ==============================
def move_all_legs(x, y):
    """Di chuyển tất cả 4 chân đến cùng vị trí"""
    for leg_id in range(4):
        move_leg(leg_id, x, y)
        time.sleep(0.1)  # Độ trễ nhỏ giữa các chân

def stand_up():
    """Đứng thẳng - tất cả chân ở vị trí trung tính"""
    print("🦿 Đứng thẳng...")
    # Vị trí trung tính: y = -15 (thấp hơn gốc), x = 0 (giữa)
    move_all_legs(0, -15)

def sit_down():
    """Ngồi xuống - tất cả chân co lại"""
    print("🦿 Ngồi xuống...")
    # Vị trí ngồi: y = -8 (cao hơn), x = 0 (giữa)
    move_all_legs(0, -8)

# ==============================
# 🧪 Test các chức năng
# ==============================
if __name__ == "__main__":
    try:
        print("🤖 Khởi động robot 4 chân...")
        time.sleep(2)
        
        # Test đứng thẳng
        stand_up()
        time.sleep(3)
        
        # Test di chuyển từng chân
        print("\n🧪 Test di chuyển từng chân:")
        test_positions = [
            (2, -12),   # Chân trước phải
            (-2, -12),  # Chân trước trái
            (2, -12),   # Chân sau phải
            (-2, -12),  # Chân sau trái
        ]
        
        for leg_id, (x, y) in enumerate(test_positions):
            print(f"\n--- Test chân {leg_id} ---")
            move_leg(leg_id, x, y)
            time.sleep(2)
        
        # Về vị trí đứng
        stand_up()
        time.sleep(2)
        
        print("\n✅ Test hoàn thành!")
        
    except KeyboardInterrupt:
        print("\n⛔ Kết thúc chương trình")
        # Đưa robot về vị trí an toàn
        try:
            stand_up()
        except:
            pass
