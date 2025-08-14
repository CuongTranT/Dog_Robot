# ------------- 2-DOF per leg control (planar x,y) -----------------
import time, math as m, numpy as np
import Adafruit_PCA9685

# ================== HÌNH HỌC & THAM SỐ ==================
L1 = 100.0  # mm - chiều dài đùi (thay bằng số của bạn)
L2 = 100.0  # mm - chiều dài cẳng (thay bằng số của bạn)

# Tư thế đứng mặc định của mỗi chân (tính theo local hip, mm)
STAND_X = 00.0
STAND_Y = -160.0  # âm = thấp hơn hip

# PCA9685
PWM_FREQ = 60
SERVO_MIN = 100
SERVO_MAX = 600

def clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)
def angle2pulse(deg): return clamp(int(100 + 500*deg/180.0), SERVO_MIN, SERVO_MAX)

pca = Adafruit_PCA9685.PCA9685(busnum=1)
pca.set_pwm_freq(PWM_FREQ)

LEG_CH = {
    'FL': (0, 1),
    'FR': (2, 3),
    'RL': (4, 5),
    'RR': (6, 7),
}

# Offset & đảo chiều (deg, bool). Điều này thay cho ma trận offset cũ 3DOF
# invert=True nghĩa là góc cơ khí tăng thì servo phải giảm (đảo chiều).
LEG_CAL = {
    'FL': {'off': ( 180,  90), 'inv': (False, False)},
    'FR': {'off': ( 0,  -90), 'inv': ( True,  True)},
    'RL': {'off': ( 180,  90), 'inv': (False, False)},
    'RR': {'off': ( 0,  -90), 'inv': ( True,  True)},
}

# ================== ĐỘNG HỌC 2R ==================
def fk_2r(theta1, theta2):
    """forward: (deg,deg) -> (x1,y1,x2,y2) theo hình của bạn"""
    t1 = m.radians(theta1)
    t2 = m.radians(theta2)
    x1 = L1*m.cos(t1); y1 = L1*m.sin(t1)
    x2 = x1 + L2*m.cos(t1 + t2)
    y2 = y1 + L2*m.sin(t1 + t2)
    return x1, y1, x2, y2

def ik_2r(x, y, elbow='down'):
    """
    inverse: (x,y)->(theta1,theta2,success)
    elbow='down' cho dáng chân khuỵu xuống (thường dùng cho quadruped)
    """
    D = (x*x + y*y - L1*L1 - L2*L2) / (2.0*L1*L2)
    if abs(D) > 1.0:
        return 0.0, 0.0, False
    s2 = m.sqrt(max(0.0, 1 - D*D))
    if elbow == 'down':
        theta2 = -m.atan2(s2, D)   # giống công thức hình bạn
    else:
        theta2 =  m.atan2(s2, D)
    # Dạng atan2 ổn định số:
    theta1 = m.atan2(y*(L1 + L2*m.cos(theta2)) - x*(L2*m.sin(theta2)),
                     x*(L1 + L2*m.cos(theta2)) + y*(L2*m.sin(theta2)))
    return m.degrees(theta1), m.degrees(theta2), True

# ================== SERVO API ==================
def set_leg_angles(leg, hip_deg, knee_deg):
    ch_hip, ch_knee = LEG_CH[leg]
    off_hip, off_knee = LEG_CAL[leg]['off']
    inv_hip, inv_knee = LEG_CAL[leg]['inv']

    # Áp offset + đảo chiều nếu cần
    a1 = ( -hip_deg if inv_hip  else hip_deg ) + off_hip
    a2 = ( -knee_deg if inv_knee else knee_deg) + off_knee

    pca.set_pwm(ch_hip,  0, angle2pulse(a1))
    pca.set_pwm(ch_knee, 0, angle2pulse(a2))

def move_foot_xy(leg, x, y, elbow='down'):
    """Điểm đặt chân (x,y) trong local hip -> servo"""
    th1, th2, ok = ik_2r(x, y, elbow)
    if not ok: 
        # ngoài workspace: bỏ qua để không bẻ gãy cơ khí
        return False
    set_leg_angles(leg, th1, th2 - th1)
    return True
# ================== POSE & QUỸ ĐẠO ==================
def stand_all():
    # for leg in ('FL','FR','RL','RR'):
    #    move_foot_xy(leg, STAND_X, STAND_Y)
    # time.sleep(0.3)
    
    move_foot_xy('RL', STAND_X, STAND_Y)

def bezier3(p0, p1, p2, p3, t):
    """Bezier bậc 3 cho đường chân mượt"""
    u = 1 - t
    return (u**3)*p0 + 3*u*u*t*p1 + 3*u*(t**2)*p2 + (t**3)*p3

def step_leg(leg, dx=60, lift=35, T=0.35, elbow='down'):
    """
    Một bước của 1 chân trong mặt phẳng:
    - dịch ra trước dx (x)
    - nhấc chân lift (y tăng lên)
    """
    x0, y0 = STAND_X, STAND_Y
    x1 = x0 + dx
    # control points cho quỹ đạo hình cánh cung
    P0 = np.array([x0, y0])
    P1 = np.array([x0+dx*0.25, y0+lift])
    P2 = np.array([x0+dx*0.75, y0+lift])
    P3 = np.array([x1, y0])

    N = 20
    for k in range(N+1):
        t = k/float(N)
        x, y = bezier3(P0, P1, P2, P3, t)
        move_foot_xy(leg, float(x), float(y), elbow)
        time.sleep(T/N)

def sweep_leg(leg, dx=60, T=0.35, elbow='down'):
    """
    Pha trụ: kéo chân còn lại về sau theo đường thẳng ở độ cao y0
    """
    x1 = STAND_X + dx
    x0 = STAND_X
    N = 20
    for k in range(N+1):
        s = k/float(N)
        x = x1*(1-s) + x0*s
        move_foot_xy(leg, x, STAND_Y, elbow)
        time.sleep(T/N)

# ================== TROT CƠ BẢN (đồng bộ FL+RR / FR+RL) ==================
def trot_forward(steps=3, dx=60, lift=35, T=0.35):
    # Nhóm 1: FL + RR nhấc; FR + RL trụ
    for _ in range(steps):
        # nhấc
        step_leg('FL', dx=dx, lift=lift, T=T)
        step_leg('RR', dx=dx, lift=lift, T=T)
        # trụ kéo về
        sweep_leg('FR', dx=dx, T=T)
        sweep_leg('RL', dx=dx, T=T)
        # đổi nhóm
        step_leg('FR', dx=dx, lift=lift, T=T)
        step_leg('RL', dx=dx, lift=lift, T=T)
        sweep_leg('FL', dx=dx, T=T)
        sweep_leg('RR', dx=dx, T=T)

# ================== DEMO ==================
if __name__ == "__main__":
    stand_all()
    # chỉnh nhanh offset/invert cho đúng cơ khí rồi hãy chạy gait
    # trot_forward(steps=2, dx=50, lift=30, T=0.30)
    # stand_all()