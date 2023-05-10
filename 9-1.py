# 캡스톤에 쓰이는 DC모터 방향조절, 거리 이동하기 

import RPi.GPIO as IO
import time
import math 

# DC 모터 왼쪽 
pwmPinA = 5 # 모터드라이버 ENA
AIN1 = 7 # IN 1
AIN2 = 16 # IN 2
encPinA = 6 # 보라색 (A) -2 
encPinB = 13 #파랑색 (B) -3

# DC 모터 오른쪽
pwmPinB = 17 # 모터드라이버 ENB
BIN3 = 14 # IN 3
BIN4 = 3 # IN 4
encPinC = 19 # 보라색 (C) 
encPinD = 26 # 파랑색 (D) 

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
        encoderPosA -= 1 
    else:
        encoderPosA += 1
   
def encoderB(encPinB):
    global encoderPosA
    if IO.input(encPinA) == IO.input(encPinB):
        encoderPosA += 1
    else:
        encoderPosA -= 1

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

# PID 제어
ratio = 360./(30 * 52); # 한 바퀴에 약 4100펄스
#rad = ratio*math.pi/180

# PID 상수
kp = float(input("KP: "))  #3.3 
kd = float(input("KD: "))  
ki = float(input("KI: "))  #0.2 

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
    target_direction = 'front' #str(input("이동하고 싶은 방향을 입력하시오 : ")) 
    target_distance = int(input("이동하고 싶은 거리를 입력하시오: "))
    
    while True: 
        wheel = 2*math.pi*r     #원둘레 = 72.26
        target_deg = 360*target_distance / wheel
        #모터기어비 추가하기 
      
      # DC 모터 왼쪽
        motorDegA = abs(encoderPosA * ratio)  
        errorA = target_deg - motorDegA
        de_A = errorA - error_prev_A
        di_A += errorA * dt
        dt = time.time() - time_prev

        motor_distanceA = motorDegA * wheel / 360
        # print(motor_distanceA)
        # print(motorDegA)
        derrorA = abs(target_distance - motor_distanceA)
        
        delta_vA = kp*de_A + ki*errorA + kd*(errorA - 2*error_prev_A + error_prev_prev_A)
        controlA += delta_vA
        error_prev_A = errorA
        error_prev_prev_A = error_prev_A
    
        # DC 모터 오른쪽
        motorDegB = abs(encoderPosB * ratio)
        errorB = target_deg - motorDegB
        de_B = errorB - error_prev_B
        di_B += errorB * dt
        dt = time.time() - time_prev

        delta_vB = kp*de_B + ki*errorB + kd*(errorB - 2*error_prev_B + error_prev_prev_B)
        controlB += delta_vB
        error_prev_B = errorB
        error_prev_prev_B = error_prev_B

        motor_distanceB = motorDegB * wheel / 360
        derrorB = abs(target_distance - motor_distanceB)
        
        # 전진 -------------------------------------------------
        if (target_direction == 'front') :  
            IO.output(AIN1, IO.HIGH)
            IO.output(AIN2, IO.LOW)
            IO.output(BIN3, IO.HIGH)
            IO.output(BIN4, IO.LOW)
            time.sleep(0.01)
            p1.ChangeDutyCycle(min(abs(controlA), 100))
            p2.ChangeDutyCycle(min(abs(controlA), 100))     #controlA 

            print('각도 = %5.1f' %(motorDegB))
            print('원하는 각도 = %5.1f' %(target_deg))
            print('ctrlA = %7.1f, degA = %5.1f, errA = %5.1f, disA = %5.1f, derrA = %5.1f' %(controlA, motorDegA, errorA, motor_distanceA, derrorA))  
            print('ctrlB = %7.1f, degB = %5.1f,s errB = %5.1f, disB = %5.1f, derrB = %5.1f' %(controlB, motorDegB, errorB, motor_distanceB, derrorB))  
            print('enc = %5.1f' %(encoderPosA))
            print('enc = %5.1f' %(encoderPosB))

            if ((motorDegA >= target_deg)): 
                IO.output(AIN1, IO.LOW)
                IO.output(AIN2, IO.LOW) 
                IO.output(BIN3, IO.LOW)
                IO.output(BIN4, IO.LOW)

                time.sleep(0.01)
                p1.ChangeDutyCycle(0)
                p2.ChangeDutyCycle(0)

            time_prev = time.time()
            time.sleep(dt_sleep)

# Crtl + c 누르면 모터 작동 멈춤
except KeyboardInterrupt: 
    pass

p1.stop()
p2.stop() 