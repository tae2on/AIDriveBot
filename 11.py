# 라이다 센서 연동  

import RPi.GPIO as IO
import time
import math 

# 핀 번호가 아니라 GPIO 번호 ! 
# DC 모터 왼쪽 (엔코더 O)
pwmPinA = 14 # 모터드라이버 ENA / ex) 핀 번호 8번, GPIO 14번 
AIN1 = 15 # IN 1
AIN2 = 18 # IN 2
encPinA = 2 # 보라색 (A)  
encPinB = 3 # 파랑색 (B)

# DC 모터 오른쪽 (엔코더 X)
pwmPinB = 17 # 모터드라이버 ENB
BIN3 = 27 # IN 3
BIN4 = 22 # IN 4
encPinC = 20 # 보라색 (C) - 20 
encPinD = 21 # 파랑색 (D) - 21
 
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

# 인터럽트
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

# 방향 설정하기 
def call():
    if( goFront == '1'):
        return go_front()
    elif( goBack == '1'):
        return go_back() 
    elif( Stop == '1'): 
        return stop()

# 전진
def go_front():
    IO.output(AIN1, IO.LOW)
    IO.output(AIN2, IO.HIGH)
    IO.output(BIN3, IO.LOW)
    IO.output(BIN4, IO.HIGH)
    time.sleep(0.01)
    p1.ChangeDutyCycle(min(abs(controlA), 100))
    p2.ChangeDutyCycle(min(abs(controlA), 100))

    print('각도 = %5.1f' %(motorDegB))  
    print('ctrlA = %7.1f, degA = %5.1f, errA = %5.1f, disA = %5.1f, derrA = %5.1f' %(controlA, motorDegA, errorA, motor_distanceA, derrorA))  
    print('ctrlB = %7.1f, degB = %5.1f,s errB = %5.1f, disB = %5.1f, derrB = %5.1f' %(controlB, motorDegB, errorB, motor_distanceB, derrorB))  

    return call()

# 후진 
def go_back():
    IO.output(AIN1, IO.HIGH)
    IO.output(AIN2, IO.LOW)
    IO.output(BIN3, IO.HIGH)
    IO.output(BIN4, IO.LOW)
    time.sleep(0.01)
    p1.ChangeDutyCycle(min(abs(controlA), 100))
    p2.ChangeDutyCycle(min(abs(controlA), 100))

    print('각도 = %5.1f' %(motorDegB))
    print('ctrlA = %7.1f, degA = %5.1f, errA = %5.1f, disA = %5.1f, derrA = %5.1f' %(controlA, motorDegA, errorA, motor_distanceA, derrorA))  
    print('ctrlB = %7.1f, degB = %5.1f,s errB = %5.1f, disB = %5.1f, derrB = %5.1f' %(controlB, motorDegB, errorB, motor_distanceB, derrorB))  

    return call () 

# 정지 
def stop():
    IO.output(AIN1, IO.LOW)
    IO.output(AIN2, IO.LOW)
    IO.output(BIN3, IO.LOW)
    IO.output(BIN4, IO.LOW)
    time.sleep(0.01)
    p1.ChangeDutyCycle(0)
    p2.ChangeDutyCycle(0)

    print('정지') 
    print('ctrlA = %7.1f, degA = %5.1f, errA = %5.1f, disA = %5.1f, derrA = %5.1f' %(controlA, motorDegA, errorA, motor_distanceA, derrorA))  
    print('ctrlB = %7.1f, degB = %5.1f,s errB = %5.1f, disB = %5.1f, derrB = %5.1f' %(controlB, motorDegB, errorB, motor_distanceB, derrorB))  

    return call()

IO.add_event_detect(encPinA, IO.BOTH, callback=encoderA)
IO.add_event_detect(encPinB, IO.BOTH, callback=encoderB)
IO.add_event_detect(encPinC, IO.BOTH, callback=encoderC)
IO.add_event_detect(encPinD, IO.BOTH, callback=encoderD)

# PID 제어
ratio = 360./264./52. # 한 바퀴에 약 13,728펄스 (정확하지 않음 - 계산값)

# PID 상수
kp = 30.0
kd = 0.         
ki = 0.

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

target_direction = 0. 
target_distance = 0.
rotation = 0.

r = 11.5    # 타이어의 반지름 

#----------------------------------- 모터의 이동 거리 -----------------------------------#
# 모터 이동 거리 (r: 11.5) 
try :
    while True: 
        wheel = 2*math.pi*r     #원둘레 = 72.26
        target_deg = (360*target_distance / wheel) / 2    # 목표 각도
    
      # DC 모터 왼쪽
        motorDegA = encoderPosA * ratio     # 모터 움직인 각도
        errorA = target_deg - motorDegA     # 각도 오차값 
        de_A = errorA - error_prev_A
        di_A += errorA * dt
        dt = time.time() - time_prev
        
        delta_vA = kp*de_A + ki*errorA + kd*(errorA - 2*error_prev_A + error_prev_prev_A)
        controlA += delta_vA
        error_prev_A = errorA
        error_prev_prev_A = error_prev_A
    
        motor_distanceA = motorDegA * wheel / 360            # 모터 움직인 거리
        derrorA = abs(target_distance - motor_distanceA)     # 거리 오차값

        # DC 모터 오른쪽
        motorDegB = abs(encoderPosB * ratio)     # 모터 움직인 각도
        errorB = target_deg - motorDegB          # 각도 오차값 
        de_B = errorB - error_prev_B
        di_B += errorB * dt
        dt = time.time() - time_prev

        delta_vB = kp*de_B + ki*errorB + kd*(errorB - 2*error_prev_B + error_prev_prev_B)
        controlB += delta_vB
        error_prev_B = errorB
        error_prev_prev_B = error_prev_B

        motor_distanceB = motorDegB * wheel / 360           # 모터 움직인 거리
        derrorB = abs(target_distance - motor_distanceB)    # 거리 오차값

        call()
        go_front()
        go_back()
        stop()

# Crtl + c 누르면 모터 작동 멈춤
except KeyboardInterrupt: 
    pass

p1.stop()
p2.stop() 