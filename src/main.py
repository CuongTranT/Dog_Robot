import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=8)

while True:
    # Từ 150 đến 180 độ kenh 4
    for angle in range(150, 180, 1):
        kit.servo[4].angle = angle
        print(f"Servo 4 -> Góc: {angle}°")
        time.sleep(0.02)
    # Từ 0 đến 80 độ kenh 5
    for angle in range(0, 80, 1):
        kit.servo[5].angle = angle
        print(f"Servo 5 -> Góc: {angle}°")
        time.sleep(0.02)
# Quay ngược lại từ 180 về 150 độ
    for angle in range(180, 150, -1):
        kit.servo[4].angle = angle
        print(f"Servo 4 <- Góc: {angle}°")
        time.sleep(0.02)

# Quay ngược kênh 5 từ 80 về 0 độ
    for angle in range(80, -1, -1):
        kit.servo[5].angle = angle
        print(f"Servo 5 <- Góc: {angle}°")
        time.sleep(0.02)



