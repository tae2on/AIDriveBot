# 360도 회전하기 

import RPi.GPIO as IO
import time

pwmPinB = 17 # 모터드라이버 ENB
BIN3 = 27 # IN 3
BIN4 = 22 # IN 4
encPinC = 20 # 보라색 (C)
encPinD = 21 # 파랑색 (D)


IO.setmode(IO.BCM)
IO.setwarnings(False)
IO.setup(encPinC, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPinD, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(pwmPinB, IO.OUT, initial=IO.LOW)
IO.setup(BIN3, IO.OUT, initial=IO.LOW)
IO.setup(BIN4, IO.OUT, initial=IO.LOW)

p = IO.PWM(17, 100)
p.start(0)

encoderPosB = 0

def encoderA(encPinC):
    global encoderPosB
    if IO.input(encPinC) == IO.input(encPinD):
        encoderPosB += 1 
    else:
        encoderPosB -= 1
   
def encoderB(encPinD):
    global encoderPosB
    if IO.input(encPinC) == IO.input(encPinD):
        encoderPosB -= 1
    else:
        encoderPosB += 1

IO.add_event_detect(encPinC, IO.BOTH, callback=encoderA)
IO.add_event_detect(encPinD, IO.BOTH, callback=encoderB)

# 원하는 각도
targetDeg = 360.

# PID 제어
ratio = 360./90./52.

# PID 상수
kp = float(input("KP:"))   #0.5
kd = float(input("KD:"))   #0.4
ki = float(input("KI:"))   #0.3

dt = 0.
di = 0.
dt_sleep = 0.01
tolerance = 0.1

start_time = time.time()
error_prev = 0.
error_prev_prev = 0.
time_prev = 0.
control = 0.
try:
    while True:
        motorDeg = encoderPosB * ratio

        error = targetDeg - motorDeg
        de = error - error_prev
        di += error * dt 
        dt = time.time() - time_prev
        # control = (kp*error) + (kd*de/dt) + (ki*di)

        # error_prev = error
        
        delta_v = kp*de + ki*error + kd*(error - 2*error_prev + error_prev_prev)
        control += delta_v
        error_prev = error
        error_prev_prev = error_prev
        
        IO.output(BIN3, control >= 0)
        IO.output(BIN4, control <= 0)
        time.sleep(0.5)
        p.ChangeDutyCycle(min(abs(control), 100))

        print('P-term = %7.1f, D-term = %7.1f, I-term = %7.1f' %(kp*error, kd*de/dt, ki*de*dt))
        print('time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' %(time.time()-start_time, encoderPosB, motorDeg, error, control))
        print('%f, %f' %(de, dt))
    
        if abs(error) <= tolerance :
            IO.output(BIN3, control >=0)
            IO.output(BIN4, control <=0)
            time.sleep(0.5)
            p.ChangeDutyCycle(0)
            break

        time_prev = time.time()
        time.sleep(dt_sleep)

    # Crtl + c 누르면 모터 작동 멈춤
except KeyboardInterrupt: 
    pass

p.stop() 