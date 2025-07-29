import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=8)

# Góc tư thế ngồi cho 1 chân (dựa trên tính toán)
servo_angles = {
    4: 40,    # Servo đùi (kênh 4)
    5: 140,    # Servo cẳng chân (kênh 5) → tạo góc ~110° với đùi
    6: 40,
    7: 140,
    8: 40,
    9: 140,
    10: 40,
    11: 140
}

def sit_one_leg():
    print("Đưa chân vào tư thế ngồi (kênh 4-5)...")
    for ch, angle in servo_angles.items():
        kit.servo[ch].angle = angle
        print(f"Servo {ch} → Góc: {angle}°")
        time.sleep(0.05)

# Gọi hàm khi bật nguồn
sit_one_leg()

# Giữ trạng thái
while True:
    time.sleep(1)
