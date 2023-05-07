# 캡스톤에 쓰이는 DC모터 360도 회전 

import RPi.GPIO as IO
import time

pwmPinB = 17 # 모터드라이버 ENB
BIN3 = 25 # IN 3
BIN4 = 3 # IN 4
encPinC = 19 # 보라색 (C) 
encPinD = 26 # 파랑색 (D)

IO.setmode(IO.BCM)
IO.setwarnings(False)
IO.setup(encPinA, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPinB, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(pwmPin, IO.OUT, initial=IO.LOW)
IO.setup(AIN1, IO.OUT, initial=IO.LOW)
IO.setup(AIN2, IO.OUT, initial=IO.LOW)

p = IO.PWM(14, 100)
p.start(0)

encoderPos = 0

def encoderA(encPinA):
    global encoderPos
    if IO.input(encPinA) == IO.input(encPinB):
        encoderPos += 1 
    else:
        encoderPos -= 1
   
def encoderB(encPinB):
    global encoderPos
    if IO.input(encPinA) == IO.input(encPinB):
        encoderPos -= 1
    else:
        encoderPos += 1

IO.add_event_detect(encPinA, IO.BOTH, callback=encoderA)
IO.add_event_detect(encPinB, IO.BOTH, callback=encoderB)

# 원하는 각도
targetDeg = 360.

# PID 제어
ratio = 360./264./52.

# PID 상수
kp = float(input("KP:"))   #0.35
kd = float(input("KD:"))   #0.
ki = float(input("KI:"))   #0.2
 
dt = 0.
di = 0.
dt_sleep = 0.01
tolerance = 0.1

start_time = time.time()
error_prev = 0.
error_prev_prev = 0.
time_prev = 0.

try:
    while True:
        motorDeg = encoderPos * ratio

        error = targetDeg - motorDeg
        de = error - error_prev
        di += error * dt 
        dt = time.time() - time_prev
        control = (kp*error) + (kd*de/dt) + (ki*di)

        error_prev = error
        
        # delta_v = kp*de + ki*error + kd*(error - 2*error_prev + error_prev_prev)
        # control += delta_v
        # error_prev = error
        # error_prev_prev = error_prev
        
        IO.output(AIN1, control >= 0)
        IO.output(AIN2, control <= 0)
        time.sleep(0.5)
        p.ChangeDutyCycle(min(abs(control), 100))

        print('P-term = %7.1f, D-term = %7.1f, I-term = %7.1f' %(kp*error, kd*de/dt, ki*de*dt))
        print('time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' %(time.time()-start_time, encoderPos, motorDeg, error, control))
        print('%f, %f' %(de, dt))
    
        if abs(error) <= tolerance :
            IO.output(AIN1, control >=0)
            IO.output(AIN2, control <=0)
            time.sleep(0.5)
            p.ChangeDutyCycle(0)
            break

        time_prev = time.time()
        time.sleep(dt_sleep)

    # Crtl + c 누르면 모터 작동 멈춤
except KeyboardInterrupt: 
    pass

p.stop() 