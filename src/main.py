import time
import sys
import termios
import tty
import select
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

# ===============================
# 🧠 Hàm đọc phím không cần Enter
# ===============================
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        if select.select([sys.stdin], [], [], 0.1)[0]:
            return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return None

# ===============================
# ⚙️ Setup PCA9685
# ===============================
i2c = busio.I2C(SCL, SDA)
pwm = PCA9685(i2c)
pwm.frequency = 50  # Servo thường dùng 50Hz

# ===============================
# 📐 Hàm chuyển đổi góc → duty_cycle
# ===============================
def angle_to_duty(angle_deg):
    # Map góc độ (0~180) thành pulse (500~2500 µs)
    pulse_us = 500 + (2000 * angle_deg / 180)
    # PCA9685 dùng 12-bit (4096 mức), 1 chu kỳ = 20ms (20000 µs)
    duty = int(pulse_us * 4096 / 20000)
    return duty

# ===============================
# 🐕 Dữ liệu góc servo theo tư thế
# ===============================
POSES = {
    'stand': [175, 65, 171, 97, 0, 75, 0, 85],
    'up':    [120, 115, 120, 110, 70, 75, 45, 45],
}

# ===============================
# 🚀 Gửi góc tới từng kênh servo
# ===============================
def set_pose(pose_name):
    if pose_name not in POSES:
        print(f"⚠️ Tư thế '{pose_name}' không tồn tại!")
        return
    angles = POSES[pose_name]
    print(f"🦾 Chuyển sang tư thế: {pose_name.upper()}")
    for ch in range(8):
        duty = angle_to_duty(angles[ch])
        pwm.channels[ch].duty_cycle = duty
    print(f"✅ Đã set servo: {angles}")

# ===============================
# 🔁 Vòng lặp chính
# ===============================
if __name__ == "__main__":
    print("🚀 Khởi động Dog Robot...")
    print("Nhấn [w] → UP | [s] → STAND | [q] → THOÁT")

    set_pose("stand")  # Tư thế mặc định ban đầu

    try:
        while True:
            key = get_key()
            if key == 'w':
                set_pose('up')
            elif key == 's':
                set_pose('stand')
            elif key == 'q':
                print("👋 Tạm biệt!")
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n🛑 Dừng bằng Ctrl+C")
