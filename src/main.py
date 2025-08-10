# set_angles.py
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

# Hiệu chuẩn dải xung (đổi nếu cần)
kit.servo[0].set_pulse_width_range(600, 2400)
kit.servo[1].set_pulse_width_range(600, 2400)

# Gán góc:
kit.servo[0].angle =  70  # độ
kit.servo[1].angle =  100  # độ
print("CH0=45°, CH1=120°")
#kenh 0: 120 độ
#kenh 1: 180 độ
#kenh 2: 110 độ
#kenh 3: 180 độ
#kenh 4: 70 độ
#kenh 5: 20 độ
#kenh 6: 70 độ
#kenh 7: 20 độ