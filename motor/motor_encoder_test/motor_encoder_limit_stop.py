# 엔코더 읽는 코드

import RPi.GPIO as IO
import time 

pwmPin = 14
AIN1 = 15
AIN2 = 18
encPinA = 2
encPinB = 3

IO.setmode(IO.BCM)
IO.setwarnings(False)
IO.setup(encPinA, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPinB, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(pwmPin,IO.OUT, initial=IO.LOW)
IO.setup(AIN1,IO.OUT, initial=IO.LOW)
IO.setup(AIN2,IO.OUT, initial=IO.LOW)

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
try : 
    while True:
        if(encoderPos >= 1000):
            IO.output(AIN1, IO.LOw)
            p.ChangeDutyCycle(0)


# Crtl + c 누르면 모터 작동 멈춤
except KeyboardInterrupt: 
    pass
