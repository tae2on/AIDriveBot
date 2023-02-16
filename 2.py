# 엔코더 읽는 코드

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

while True:
    IO.output(AIN1, IO.HIGH)
    IO.output(AIN2, IO.LOW)
    IO.output(BIN3, IO.LOW)
    IO.output(BIN4, IO.HIGH)
    time.sleep(0.01)
    p1.ChangeDutyCycle(101)
    p2.ChangeDutyCycle(101)     #controlA 

    print('전진')
