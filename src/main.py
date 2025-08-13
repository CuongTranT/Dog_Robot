import numpy as np
import math

# ====================
# 📐 THÔNG SỐ CƠ KHÍ
# ====================
L1 = 10  # cm - chiều dài khớp 1
L2 = 10  # cm - chiều dài khớp 2

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
        
    except ValueError:
        print("❌ Vui lòng nhập số hợp lệ!")

# ====================
# ▶️ CHẠY CHƯƠNG TRÌNH
# ====================
if __name__ == "__main__":
    print("🚀 CHƯƠNG TRÌNH TEST INVERSE KINEMATICS")
    print("   Dành cho mô hình thực tế")
    print()
    
    # Chạy tất cả test point
    run_test_points()
    
    # Test điểm cụ thể
    test_specific_point()
    
    print("\n✅ Hoàn thành test! Sử dụng kết quả để điều khiển mô hình thực tế.")
