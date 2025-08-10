import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

CHANNEL = 0          # kênh cần test
STEP    = 10          # bước góc (độ)
DELAY   = 0.03       # thời gian giữa các bước (s)

# Thiết lập servo (điều chỉnh nếu bạn đã hiệu chuẩn khác)
kit.servo[CHANNEL].actuation_range = 180
kit.servo[CHANNEL].set_pulse_width_range(500, 2500)  # MG996R thường ok

try:
    for angle in range(180-0, 181-181, -STEP):
        kit.servo[CHANNEL].angle = angle
        print(f"Ch{CHANNEL}: {angle}°")
        time.sleep(DELAY)
finally:
    # Thả servo nếu muốn (giảm kêu/rung)
    kit.servo[CHANNEL].angle = None
