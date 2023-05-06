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
targetDeg = 1080.

# PID 제어
ratio = 360. / (84 * 10) # 한 바퀴에 약 1350펄스 (정확하지 않음 - 계산값)
 
# PID 상수
''' kp의 값은 0.1 ~ 0.5 사이 
    ki의 값은 0.001 ~ 0.1 사이 '''
kp = float(input("kp: ")) #2.0
ki = float(input("ki: ")) # 0.001
kd = float(input("kd: "))         


max_i = 100  # 최대 I 값
min_i = -100 # 최소 I 값

dt = 0.001
di = 0.
dt_sleep = 0.0001
tolerance = 0.1

#start_time = time.time()
error_prev = 0.
error_prev_prev = 0.
time_prev = 0.

try:
    while True:
        motorDeg = abs(encoderPosA * ratio)

        error = targetDeg - motorDeg
        de = error - error_prev
        di = max(min_i, min(max_i, di + error * dt)) # 최대 I, 최소 I 지정
        time_now = time.time()
        dt = time.time() - time_prev
        control = (kp*error) + (kd*de/dt) + (ki*di)

        error_prev = error
        time_prev = time_now

        IO.output(AIN1, control >= 0)
        IO.output(AIN2, control <= 0)
        time.sleep(0.001) 
        p1.ChangeDutyCycle(min(abs(control), 100.))

        print('enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' %(encoderPosA, motorDeg, error, control))
              
        if (motorDeg >= 1080) :
            IO.output(AIN1, control >=0)
            IO.output(AIN2, control <=0)
            p1.ChangeDutyCycle(0)
            break

        time_prev = time.time()
        time.sleep(dt_sleep)

    # Crtl + c 누르면 모터 작동 멈춤
except KeyboardInterrupt: 
    pass 

p1.stop() 
