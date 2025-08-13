import time
import sys
import termios
import tty
import select
from Adafruit_PCA9685 import PCA9685

# ===============================
# ⚙️ Khởi tạo PCA9685
pwm = PCA9685(address=0x40, busnum=1)
pwm.set_pwm_freq(60)  # Tần số phù hợp servo MG996R

# ===============================
# 📐 Chuyển góc độ sang xung PWM
def angle_to_pwm(angle):
    pulse_min = 500    # us (0°)
    pulse_max = 2500   # us (180°)
    pulse_us = pulse_min + (pulse_max - pulse_min) * angle / 180
    pulse_len = 1000000.0 / 60 / 4096  # 60Hz: chu kỳ 16.66ms → độ dài 1 bước
    pulse = int(pulse_us / pulse_len)
    return pulse

# ===============================
# 🧠 Danh sách tư thế robot dog
POSES = {
    "stand": [175, 65, 171, 97, 0, 75, 0, 85],
    "up":    [120, 115, 120, 110, 70, 75, 45, 45],
}

# ===============================
# 🧠 Hàm đọc phím từ terminal (non-block)
def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        if select.select([sys.stdin], [], [], 0.05)[0]:
            return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return None

# ===============================
# 🚀 Gửi góc tới servo
def set_pose(pose_name):
    if pose_name not in POSES:
        print(f"⚠️ Không có tư thế '{pose_name}'")
        return
    angles = POSES[pose_name]
    print(f"🦴 Chuyển sang tư thế: {pose_name.upper()}")
    for ch in range(8):
        pulse = angle_to_pwm(angles[ch])
        pwm.set_pwm(ch, 0, pulse)
    print(f"✅ Góc servo: {angles}")

# ===============================
# 🔁 Vòng lặp chính
if __name__ == "__main__":
    print("🐶 Robot Dog Control - PCA9685")
    print("⏎ Phím [w] → UP | [s] → STAND | [q] → QUIT")

    set_pose("stand")

    try:
        while True:
            key = get_key()
            if key == 'w':
                set_pose("up")
            elif key == 's':
                set_pose("stand")
            elif key == 'q':
                print("👋 Tạm biệt!")
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n🛑 Dừng lại (Ctrl+C)")
