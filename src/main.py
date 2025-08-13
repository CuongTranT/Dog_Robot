import time
import sys
import termios
import tty
import select
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

# ===============================
# ⚙️ Hàm đọc phím không cần enter
# ===============================
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        if select.select([sys.stdin], [], [], 0.05)[0]:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

# ===============================
# ⚙️ Khởi tạo PWM PCA9685
# ===============================
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 60

# ===============================
# ⚙️ Cấu hình xung PWM cho servo
# ===============================
SERVO_MIN = 500   # microsecond (0 độ)
SERVO_MAX = 2500  # microsecond (180 độ)

def angle_to_pwm(angle):
    us = SERVO_MIN + (SERVO_MAX - SERVO_MIN) * angle / 180
    pwm_val = int(us * 4096 / 20000)  # 20ms chu kỳ @50Hz
    return pwm_val

# ===============================
# ⚙️ Các tư thế
# ===============================
POSES = {
    "stand": [175, 65, 171, 97, 0, 75, 0, 85],
    "up":    [120, 115, 120, 110, 70, 75, 45, 45],
}

def set_pose(name):
    if name not in POSES:
        print(f"Không tìm thấy tư thế '{name}'")
        return
    angles = POSES[name]
    for ch in range(8):
        pwm_val = angle_to_pwm(angles[ch])
        pca.channels[ch].duty_cycle = pwm_val
    print(f"Đã chuyển sang tư thế: {name.upper()}")

# ===============================
# 🧠 Chương trình chính
# ===============================
if __name__ == "__main__":
    print("🐶 Khởi động Dog Robot...")
    set_pose("stand")
    print("Nhấn [w] để UP, [s] để STAND, [q] để thoát")

    while True:
        key = get_key()
        if key == 'w':
            set_pose("up")
        elif key == 's':
            set_pose("stand")
        elif key == 'q':
            print("Kết thúc.")
            break
        time.sleep(0.1)
