# DC 모터 2개 Control

import RPi.GPIO as IO
import time

# DC 모터 왼쪽
pwmPinA = 14 # 모터드라이버 ENA
AIN1 = 15 # IN 1
AIN2 = 18 # IN 2
encPinA = 2 # 보라색 (A)
encPinB = 3 # 파랑색 (B)

# DC 모터 오른쪽
pwmPinB = 17 # 모터드라이버 ENB
BIN3 = 27 # IN 3
BIN4 = 22 # IN 4
encPinC = 20 # 보라색 (C)
encPinD = 21 # 파랑색 (D)

IO.setmode(IO.BCM)
IO.setwarnings(False)
IO.setup(encPinA, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPinB, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPinC, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPinD, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(pwmPinA, IO.OUT, initial=IO.LOW)
IO.setup(pwmPinB, IO.OUT, initial=IO.LOW)
IO.setup(AIN1, IO.OUT, initial=IO.LOW)
IO.setup(AIN2, IO.OUT, initial=IO.LOW)
IO.setup(BIN3, IO.OUT, initial=IO.LOW)
IO.setup(BIN4, IO.OUT, initial=IO.LOW)

p1 = IO.PWM(pwmPinA, 100)
p2 = IO.PWM(pwmPinB, 100)
p1.start(0)
p2.start(0)

encoderPosA = 0
encoderPosB = 0

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

def encoderC(encPinC):
    global encoderPosB
    if IO.input(encPinC) == IO.input(encPinD):
        encoderPosB += 1 
    else:
        encoderPosB -= 1
   
def encoderD(encPinD):
    global encoderPosB
    if IO.input(encPinC) == IO.input(encPinD):
        encoderPosB -= 1
    else:
        encoderPosB += 1

IO.add_event_detect(encPinA, IO.BOTH, callback=encoderA)
IO.add_event_detect(encPinB, IO.BOTH, callback=encoderB)
IO.add_event_detect(encPinC, IO.BOTH, callback=encoderC)
IO.add_event_detect(encPinD, IO.BOTH, callback=encoderD)

# 원하는 각도
targetDeg = 360.

# PID 제어
ratio = 360./264./52.

# PID 상수
kp = float(input("KP: "))   #0.35
kd = float(input("KD: "))   #0.
ki = float(input("KI: "))   #0.2

# DC 모터 왼쪽
di_A = 0.
error_prev_A = 0.
error_prev_prev_A = 0.

# DC 모터 오른쪽
di_B = 0.
error_prev_B = 0.
error_prev_prev_B = 0.

dt = 0.
dt_sleep = 0.01
tolerance = 0.1

start_time = time.time()

time_prev = 0.
controlA = 0.
controlB = 0.

try:
    while True:
        # DC 모터 왼쪽
        motorDegA = encoderPosA * ratio

        errorA = targetDeg - motorDegA
        de_A = errorA - error_prev_A
        di_A += errorA * dt
        dt = time.time() - time_prev

        delta_vA = kp*de_A + ki*errorA + kd*(errorA - 2*error_prev_A + error_prev_prev_A)
        controlA += delta_vA
        error_prev_A = errorA
        error_prev_prev_A = error_prev_A

        # DC 모터 오른쪽
        motorDegB = -(encoderPosB * ratio)

        errorB = targetDeg - motorDegB
        de_B = errorB - error_prev_B
        di_B += errorB * dt
        dt = time.time() - time_prev

        delta_vB = kp*de_B + ki*errorB + kd*(errorB - 2*error_prev_B + error_prev_prev_B)
        controlB += delta_vB
        error_prev_B = errorB
        error_prev_prev_B = error_prev_B

        IO.output(AIN1, IO.LOW)
        IO.output(AIN2, IO.HIGH)
        IO.output(BIN3, IO.LOW)
        IO.output(BIN4, IO.HIGH)
        time.sleep(0.01)
        p1.ChangeDutyCycle(min(abs(controlA), 100))
        p2.ChangeDutyCycle(min(abs(controlB), 100))

        print('encA = %d, degA = %5.1f, errA = %5.1f, ctrlA = %7.1f' %(encoderPosA, motorDegA, errorA, controlA))
        print('encB = %d, degB = %5.1f, errB = %5.1f, ctrlB = %7.1f' %(encoderPosB, motorDegB, errorB, controlB))
    
        if (motorDegA >= targetDeg) & (controlA <= 0):
            IO.output(AIN1, IO.LOW)
            IO.output(AIN2, IO.LOW) 
            IO.output(BIN3, IO.LOW)
            IO.output(BIN4, IO.LOW)

            time.sleep(0.01)
            p1.ChangeDutyCycle(0)
            p2.ChangeDutyCycle(0)
            print('stopA')

        time_prev = time.time()
        time.sleep(dt_sleep)

# Crtl + c 누르면 모터 작동 멈춤
except KeyboardInterrupt: 
    pass

p1.stop()
p2.stop() 