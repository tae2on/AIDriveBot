# 엔코더 읽는 코드

import RPi.GPIO as IO
import time 

pwmPin = 12
AIN1 = 16
AIN2 = 25
encPinA = 23
encPinB = 24

IO.setmode(IO.BCM)
IO.setwarnings(False)
IO.setup(encPinA, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPinB, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(pwmPin,IO.OUT, initial=IO.LOW)
IO.setup(AIN1,IO.OUT, initial=IO.LOW)
IO.setup(AIN2,IO.OUT, initial=IO.LOW)

p = IO.PWM(12, 100)
p.start(0)

encoderPos = 0

def encoderA(encPinA):
    global encoderPos
    if IO.input(encPinA) == IO.input(encPinB):
        encoderPos -= 1 
    else:
        encoderPos += 1
   
def encoderB(encPinB):
    global encoderPos
    if IO.input(encPinA) == IO.input(encPinB):
        encoderPos -= 1
    else:
        encoderPos += 1

IO.add_event_detect(encPinA, IO.BOTH, callback=encoderA)
IO.add_event_detect(encPinB, IO.BOTH, callback=encoderB)
try:
    while True:
        # 엔코더 값 읽어오기
        IO.output(AIN1, IO.HIGH)
        p.ChangeDutyCycle(100)
        time.sleep(0.1)
        print(encoderPos, flush=True)

        if(encoderPos >= 1500):
            IO.output(AIN1, IO.LOW)
            p.ChangeDutyCycle(0)
            print(encoderPos, flush=True)
            break # 코드 멈추기
except KeyboardInterrupt:
    p.ChangeDutyCycle(0)

    print("Measurement stopped by user")