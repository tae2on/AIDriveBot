import RPi.GPIO as IO
import time
import math 

# DC 모터 왼쪽 (엔코더 O)
pwmPinA = 14 # 모터드라이버 ENA
AIN1 = 15 # IN 1
AIN2 = 18 # IN 2
encPinA = 2 # 보라색 (A)  
encPinB = 3 # 파랑색 (B)

IO.setmode(IO.BCM)
IO.setwarnings(False)
IO.setup(encPinA, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPinB, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(pwmPinA, IO.OUT, initial=IO.LOW)
IO.setup(AIN1, IO.OUT, initial=IO.LOW)
IO.setup(AIN2, IO.OUT, initial=IO.LOW)

p1 = IO.PWM(pwmPinA, 100)
p1.start(0)

encoderPosA = 0

def encoderA(encPinA):
    global encoderPosA
    if IO.input(encPinA) == IO.input(encPinB):
        encoderPosA += 1 
    else:
        encoderPosA -= 1
   
def encoderB(encPinB):
    global encoderPosA
    if IO.input(encPinA) == IO.input(encPinB):
        encoderPosA -= 1
    else:
        encoderPosA += 1

IO.add_event_detect(encPinA, IO.BOTH, callback=encoderA)
IO.add_event_detect(encPinB, IO.BOTH, callback=encoderB)

# 원하는 각도
targetDeg = 360.

# PID 제어
ratio = 360./144./9.027 # 한 바퀴에 약 1350펄스 (정확하지 않음 - 계산값)

# PID 상수
kp = float(input("kp: "))
kd = float(input("kd: "))         
ki = float(input("ki: "))

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
        motorDeg = abs(encoderPosA * ratio)

        error = targetDeg - motorDeg
        de = error - error_prev
        di += error * dt 
        dt = time.time() - time_prev
        control = (kp*error) + (kd*de/dt) + (ki*di)

        error_prev = error

        IO.output(AIN1, control >= 0)
        IO.output(AIN2, control <= 0)
        time.sleep(0.5)
        p1.ChangeDutyCycle(min(abs(control), 35))

        print('time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' %(time.time()-start_time, encoderPosA, motorDeg, error, control))
        print('enc = %6.3f' %encoderPosA)
    
        if abs(error) <= tolerance :
            IO.output(AIN1, control >=0)
            IO.output(AIN2, control <=0)
            time.sleep(0.5)
            p1.ChangeDutyCycle(0)
            break

        time_prev = time.time()
        time.sleep(dt_sleep)

    # Crtl + c 누르면 모터 작동 멈춤
except KeyboardInterrupt: 
    pass

p1.stop() 
