# start_mode.py
from adafruit_servokit import ServoKit

# Góc khởi tạo (độ) cho các kênh 0..7
INIT_ANGLES = [140, 180, 110, 180, 70, 20, 85, 20]

# Biên độ xung cho servo (điều chỉnh nếu cần)
MIN_US, MAX_US = 600, 2400

def start_mode():
    kit = ServoKit(channels=16)
    for ch in range(8):
        kit.servo[ch].set_pulse_width_range(MIN_US, MAX_US)
        kit.servo[ch].angle = INIT_ANGLES[ch]
    print("Applied start pose:", INIT_ANGLES)

if __name__ == "__main__":
    start_mode()
