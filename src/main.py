import numpy as np
import math
import time

# Import the PCA9685 module
import Adafruit_PCA9685

# ====================
# 📐 THÔNG SỐ CƠ KHÍ
# ====================
L1 = 10  # cm - chiều dài khớp 1
L2 = 10  # cm - chiều dài khớp 2

# ====================
# ⚙️ CẤU HÌNH SERVO
# ====================
# Khởi tạo PCA9685 (chỉ định busnum=1 cho Raspberry Pi)
pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
pwm.set_pwm_freq(60)  # Tần số 60Hz

# Cấu hình kênh servo
HIP_CHANNEL = 0      # Kênh 0 - Khớp hông
KNEE_CHANNEL = 1     # Kênh 1 - Khớp gối

# Cấu hình PWM cho servo (0-180° → 150-600)
SERVO_MIN = 150      # Góc 0°
SERVO_MAX = 600      # Góc 180°

# Số lượng channel tối đa để test
MAX_CHANNELS = 8

def angle_to_pwm(angle_deg):
    """
    Chuyển đổi góc (độ) sang giá trị PWM
    
    Args:
        angle_deg: Góc tính bằng độ (0-180)
    
    Returns:
        int: Giá trị PWM (150-600)
    """
    # Giới hạn góc trong khoảng 0-180°
    angle_deg = max(0, min(180, angle_deg))
    
    # Chuyển đổi góc sang PWM
    pwm_value = int(SERVO_MIN + (SERVO_MAX - SERVO_MIN) * angle_deg / 180.0)
    return pwm_value

def set_servo_angle(channel, angle_deg):
    """
    Đặt góc cho servo
    
    Args:
        channel: Kênh servo (0-7)
        angle_deg: Góc tính bằng độ
    """
    pwm_value = angle_to_pwm(angle_deg)
    pwm.set_pwm(channel, 0, pwm_value)
    print(f"Kênh {channel}: Góc {angle_deg:.1f}° → PWM {pwm_value}")

def test_all_channels():
    """
    Test tất cả 8 channel lần lượt
    """
    print("\n🧪 TEST TẤT CẢ 8 CHANNEL")
    print("=" * 50)
    
    # Test từng channel một
    for channel in range(MAX_CHANNELS):
        print(f"\n--- Test Channel {channel} ---")
        
        # Đặt về vị trí giữa
        set_servo_angle(channel, 90)
        time.sleep(0.5)
        
        # Test góc nhỏ
        set_servo_angle(channel, 45)
        time.sleep(0.5)
        
        # Test góc lớn
        set_servo_angle(channel, 135)
        time.sleep(0.5)
        
        # Về vị trí giữa
        set_servo_angle(channel, 90)
        time.sleep(0.5)
        
        print(f"✅ Hoàn thành test Channel {channel}")
    
    print("\n🎉 Đã test xong tất cả 8 channel!")

def test_specific_channel(channel):
    """
    Test một channel cụ thể
    
    Args:
        channel: Số channel (0-7)
    """
    if channel < 0 or channel >= MAX_CHANNELS:
        print(f"❌ Channel {channel} không hợp lệ! Phải từ 0-{MAX_CHANNELS-1}")
        return
    
    print(f"\n🎯 TEST CHANNEL {channel}")
    print("=" * 30)
    
    # Test các góc cơ bản
    angles = [90, 45, 135, 90, 0, 180, 90]
    
    for i, angle in enumerate(angles):
        print(f"Bước {i+1}: Góc {angle}°")
        set_servo_angle(channel, angle)
        time.sleep(0.3)
    
    print(f"✅ Hoàn thành test Channel {channel}")

def test_channel_sequence():
    """
    Test các channel theo thứ tự tuần tự
    """
    print("\n🔄 TEST CHANNEL THEO THỨ TỰ")
    print("=" * 40)
    
    # Test từng channel một cách tuần tự
    for channel in range(MAX_CHANNELS):
        print(f"\n📡 Test Channel {channel}")
        set_servo_angle(channel, 90)  # Vị trí giữa
        time.sleep(0.2)
    
    print("\n✅ Đã test tuần tự tất cả channel!")

def reset_all_channels():
    """
    Đặt tất cả channel về vị trí giữa (90°)
    """
    print("\n🔄 ĐẶT TẤT CẢ CHANNEL VỀ VỊ TRÍ GIỮA")
    print("=" * 50)
    
    for channel in range(MAX_CHANNELS):
        set_servo_angle(channel, 90)
        time.sleep(0.1)
    
    print("✅ Đã đặt tất cả channel về 90°")

def move_leg_to_position(x, y):
    """
    Di chuyển chân đến vị trí (x, y) sử dụng inverse kinematics
    
    Args:
        x, y: Tọa độ điểm cuối (cm)
    
    Returns:
        bool: True nếu thành công, False nếu thất bại
    """
    print(f"\n🎯 Di chuyển chân đến vị trí ({x}, {y})")
    
    # Tính toán góc khớp
    theta1, theta2, success = inverse_kinematics(x, y)
    
    if not success:
        print("❌ Không thể đến vị trí này!")
        return False
    
    print(f"✓ Tính toán thành công:")
    print(f"  θ₁ (khớp hông) = {theta1:.2f}°")
    print(f"  θ₂ (khớp gối)  = {theta2:.2f}°")
    print(f"  β = θ₁ + θ₂    = {theta1 + theta2:.2f}°")
    
    # Chuyển đổi góc cho servo (có thể cần điều chỉnh tùy cấu hình cơ khí)
    hip_angle = theta1
    knee_angle = theta2
    
    # Giới hạn góc servo (0-180°)
    hip_angle = max(0, min(180, hip_angle))
    knee_angle = max(0, min(180, knee_angle))
    
    print(f"\n🔧 Điều khiển servo:")
    print(f"  Kênh {HIP_CHANNEL} (Hip): {hip_angle:.1f}°")
    print(f"  Kênh {KNEE_CHANNEL} (Knee): {knee_angle:.1f}°")
    
    # Di chuyển servo
    set_servo_angle(HIP_CHANNEL, hip_angle)
    time.sleep(0.1)  # Đợi servo ổn định
    set_servo_angle(KNEE_CHANNEL, knee_angle)
    
    print("✅ Hoàn thành di chuyển!")
    return True

def test_servo_movement():
    """
    Test chuyển động servo cơ bản
    """
    print("\n🧪 TEST CHUYỂN ĐỘNG SERVO")
    print("=" * 40)
    
    # Test khớp hông (kênh 0)
    print("Test khớp hông (kênh 0):")
    set_servo_angle(HIP_CHANNEL, 90)   # Góc giữa
    time.sleep(1)
    set_servo_angle(HIP_CHANNEL, 45)   # Góc nhỏ
    time.sleep(1)
    set_servo_angle(HIP_CHANNEL, 135)  # Góc lớn
    time.sleep(1)
    set_servo_angle(HIP_CHANNEL, 90)   # Về giữa
    time.sleep(1)
    
    # Test khớp gối (kênh 1)
    print("\nTest khớp gối (kênh 1):")
    set_servo_angle(KNEE_CHANNEL, 90)  # Góc giữa
    time.sleep(1)
    set_servo_angle(KNEE_CHANNEL, 45)  # Góc nhỏ
    time.sleep(1)
    set_servo_angle(KNEE_CHANNEL, 135) # Góc lớn
    time.sleep(1)
    set_servo_angle(KNEE_CHANNEL, 90)  # Về giữa
    time.sleep(1)
    
    print("✅ Hoàn thành test servo!")

# ====================
# 🔧 HÀM ĐỘNG HỌC NGHỊCH
# ====================
def inverse_kinematics(x, y):
    """
    Tính toán góc khớp từ vị trí điểm cuối
    Sử dụng công thức mới: D = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2)
    
    Args:
        x, y: Tọa độ điểm cuối (cm)
    
    Returns:
        tuple: (theta1_deg, theta2_deg, success)
        - theta1_deg: Góc khớp hông (độ)
        - theta2_deg: Góc khớp gối (độ) 
        - success: True nếu tính toán thành công
    """
    # Công thức mới: D = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2)
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    
    # Kiểm tra khả năng đạt được
    if abs(D) > 1:
        print(f"❌ Vị trí ({x}, {y}) không thể đạt được! D = {D:.3f}")
        return 0, 0, False
    
    # theta2 = -atan2(sqrt(1 - D^2), D) - gập khớp
    theta2 = -math.atan2(math.sqrt(1 - D**2), D)
    
    # theta1 = atan2(y*(L1 + L2*cos(theta2)) - x*L2*sin(theta2), 
    #                x*(L1 + L2*cos(theta2)) + y*L2*sin(theta2))
    theta1 = math.atan2(y*(L1 + L2*math.cos(theta2)) - x*L2*math.sin(theta2),
                        x*(L1 + L2*math.cos(theta2)) + y*L2*math.sin(theta2))
    
    # Chuyển đổi sang độ
    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)
    
    return theta1_deg, theta2_deg, True

def forward_kinematics(theta1_deg, theta2_deg):
    """
    Kiểm tra động học thuận để xác nhận kết quả
    
    Args:
        theta1_deg, theta2_deg: Góc khớp (độ)
    
    Returns:
        tuple: (x, y) - tọa độ điểm cuối
    """
    theta1 = math.radians(theta1_deg)
    theta2 = math.radians(theta2_deg)
    
    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)
    x2 = x1 + L2 * math.cos(theta1 + theta2)
    y2 = y1 + L2 * math.sin(theta1 + theta2)
    
    return x2, y2

def test_inverse_kinematics(x, y):
    """
    Test công thức inverse kinematics tại một điểm
    
    Args:
        x, y: Tọa độ test (cm)
    """
    print(f"\n=== TEST TẠI ĐIỂM ({x}, {y}) ===")
    
    # Tính inverse kinematics
    theta1, theta2, success = inverse_kinematics(x, y)
    
    if success:
        print(f"✓ Thành công!")
        print(f"  θ₁ (khớp hông) = {theta1:.2f}°")
        print(f"  θ₂ (khớp gối)  = {theta2:.2f}°")
        print(f"  β = θ₁ + θ₂    = {theta1 + theta2:.2f}°")
        
        # Kiểm tra bằng forward kinematics
        x_check, y_check = forward_kinematics(theta1, theta2)
        print(f"  Kiểm tra FK: ({x_check:.3f}, {y_check:.3f})")
        print(f"  Sai số: Δx = {abs(x_check - x):.4f}, Δy = {abs(y_check - y):.4f}")
        
        return theta1, theta2, True
    else:
        print("✗ Không thể đến điểm này!")
        return 0, 0, False

# ====================
# 🎯 CÁC TEST POINT
# ====================
def run_test_points():
    """
    Chạy tất cả test point để kiểm tra công thức
    """
    print("=" * 50)
    print("🧪 TEST INVERSE KINEMATICS - MÔ HÌNH THỰC TẾ")
    print("=" * 50)
    print(f"Thông số cơ khí: L1 = {L1}cm, L2 = {L2}cm")
    print("Trục Y hướng xuống dưới")
    print("=" * 50)
    
    # Danh sách test point
    test_points = [
        # Các điểm trong workspace
        (10, 10),   # Điểm A từ code gốc
    ]
    
    successful_tests = 0
    total_tests = len(test_points)
    
    for i, (x, y) in enumerate(test_points, 1):
        print(f"\n{i:2d}. ", end="")
        theta1, theta2, success = test_inverse_kinematics(x, y)
        if success:
            successful_tests += 1
    
    print("\n" + "=" * 50)
    print(f"📊 KẾT QUẢ TỔNG KẾT:")
    print(f"   Thành công: {successful_tests}/{total_tests}")
    print(f"   Tỷ lệ: {successful_tests/total_tests*100:.1f}%")
    print("=" * 50)

def test_specific_point():
    """
    Test một điểm cụ thể - bạn có thể thay đổi tọa độ ở đây
    """
    print("\n🎯 TEST ĐIỂM CỤ THỂ")
    print("Nhập tọa độ để test:")
    
    try:
        x = float(input("X (cm): "))
        y = float(input("Y (cm): "))
        
        theta1, theta2, success = test_inverse_kinematics(x, y)
        
        if success:
            print(f"\n📋 KẾT QUẢ CHO MÔ HÌNH THỰC TẾ:")
            print(f"   Đặt servo khớp hông: {theta1:.1f}°")
            print(f"   Đặt servo khớp gối:  {theta2:.1f}°")
            print(f"   Góc tổng:            {theta1 + theta2:.1f}°")
            
            # Hỏi có muốn di chuyển thực tế không
            move_real = input("\n🤖 Có muốn di chuyển chân thực tế? (y/n): ").lower()
            if move_real == 'y':
                move_leg_to_position(x, y)
        
    except ValueError:
        print("❌ Vui lòng nhập số hợp lệ!")

def interactive_control():
    """
    Điều khiển tương tác cho mô hình thực tế
    """
    print("\n🎮 ĐIỀU KHIỂN TƯƠNG TÁC")
    print("=" * 40)
    
    while True:
        print("\nChọn chức năng:")
        print("1. Test chuyển động servo cơ bản")
        print("2. Test tất cả 8 channel")
        print("3. Test channel cụ thể")
        print("4. Test channel tuần tự")
        print("5. Reset tất cả channel về 90°")
        print("6. Di chuyển đến vị trí cụ thể")
        print("7. Điều khiển servo trực tiếp")
        print("8. Thoát")
        
        choice = input("Nhập lựa chọn (1-8): ")
        
        if choice == '1':
            test_servo_movement()
        elif choice == '2':
            test_all_channels()
        elif choice == '3':
            try:
                channel = int(input(f"Nhập số channel (0-{MAX_CHANNELS-1}): "))
                test_specific_channel(channel)
            except ValueError:
                print("❌ Vui lòng nhập số nguyên!")
        elif choice == '4':
            test_channel_sequence()
        elif choice == '5':
            reset_all_channels()
        elif choice == '6':
            try:
                x = float(input("X (cm): "))
                y = float(input("Y (cm): "))
                move_leg_to_position(x, y)
            except ValueError:
                print("❌ Vui lòng nhập số hợp lệ!")
        elif choice == '7':
            try:
                channel = int(input(f"Channel (0-{MAX_CHANNELS-1}): "))
                angle = float(input("Góc (0-180°): "))
                
                if 0 <= channel < MAX_CHANNELS:
                    set_servo_angle(channel, angle)
                else:
                    print(f"❌ Channel {channel} không hợp lệ!")
                
            except ValueError:
                print("❌ Vui lòng nhập số hợp lệ!")
        elif choice == '8':
            print("👋 Tạm biệt!")
            break
        else:
            print("❌ Lựa chọn không hợp lệ!")

# ====================
# ▶️ CHẠY CHƯƠNG TRÌNH
# ====================
if __name__ == "__main__":
    print("🚀 CHƯƠNG TRÌNH ĐIỀU KHIỂN CHÂN ROBOT 2 DOF")
    print("   Sử dụng Adafruit_PCA9685 - Test 8 Channel")
    print(f"   Kênh 0: Hip, Kênh 1: Knee, Tổng cộng: {MAX_CHANNELS} channel")
    print()
    
    try:
        # Chạy tất cả test point
        run_test_points()
        
        # Test điểm cụ thể
        test_specific_point()
        
        # Điều khiển tương tác
        interactive_control()
        
    except KeyboardInterrupt:
        print("\n\n⚠️ Chương trình bị dừng bởi người dùng")
        print("🔒 Đặt tất cả channel về vị trí an toàn...")
        
        # Đặt tất cả channel về vị trí an toàn (90°)
        for channel in range(MAX_CHANNELS):
            set_servo_angle(channel, 90)
            time.sleep(0.1)
        
        print("✅ Đã đặt tất cả channel về vị trí an toàn!")
    
    print("\n✅ Hoàn thành! Sử dụng kết quả để điều khiển mô hình thực tế.")
