# 모터 속도 측정하기 (RPM, deg/s) 

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

IO.output(AIN1, IO.HIGH)
# IO.output(AIN2, IO.LOW)
p.ChangeDutyCycle(100)

print('회전')
    
time.sleep(0.5)

