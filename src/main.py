import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=8)

while True:
    # Từ 100 đến 180 độ
    for angle in range(150, 181, 1):
        kit.servo[4].angle = angle
        print(f"Servo 4 -> Góc: {angle}°")
        time.sleep(0.02)

    # Quay ngược lại từ 180 về 100 độ
    for angle in range(180, 150, -1):
        kit.servo[4].angle = angle
        print(f"Servo 4 <- Góc: {angle}°")
        time.sleep(0.02)
            # Từ 100 đến 180 độ
    for angle in range(100, 181, 1):
        kit.servo[5].angle = angle
        print(f"Servo 5 -> Góc: {angle}°")
        time.sleep(0.02)

    # Quay ngược lại từ 180 về 100 độ
    for angle in range(180, 99, -1):
        kit.servo[5].angle = angle
        print(f"Servo 5 <- Góc: {angle}°")
        time.sleep(0.02)
