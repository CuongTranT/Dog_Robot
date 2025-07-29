import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=8)

while True:
    # Từ 150 đến 180 độ kenh 4
    # Từ 0 đến 80 độ kenh 5
    for angle in range(0, 80, 1):
        # kit.servo[4].angle = angle
        kit.servo[5].angle = angle
        # print(f"Servo 4 -> Góc: {angle}°")
        print(f"Servo 5 -> Góc: {angle}°")
        time.sleep(0.02)

    # Quay ngược lại từ 180 về 100 độ
    for angle in range(80, 0, -1):
        # kit.servo[4].angle = angle
        kit.servo[5].angle = angle
        # print(f"Servo 4 <- Góc: {angle}°")
        print(f"Servo 5 -> Góc: {angle}°")
        time.sleep(0.02)


