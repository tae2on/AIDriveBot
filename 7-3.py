import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

pwmPinA = 14 # 모터드라이버 ENA
AIN1 = 15 # IN 1
AIN2 = 18 # IN 2
encPinA = 2 # 보라색 (A)  
encPinB = 3 # 파랑색 (B)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(pwmPinA, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(AIN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(AIN2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(encPinA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encPinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)

p1 = GPIO.PWM(pwmPinA, 100)
encoderPosA = 0

def encoderA(encPinA):
    global encoderPosA
    if GPIO.input(encPinA) == GPIO.input(encPinB):
        encoderPosA += 1
    else:
        encoderPosA -= 1

def encoderB(encPinB):
    global encoderPosA
    if GPIO.input(encPinA) == GPIO.input(encPinB):
        encoderPosA -= 1
    else:
        encoderPosA += 1

GPIO.add_event_detect(encPinA, GPIO.BOTH, callback=encoderA)
GPIO.add_event_detect(encPinB, GPIO.BOTH, callback=encoderB)

while True:
    GPIO.output(AIN1, GPIO.HIGH)
    #GPIO.output(AIN2, GPIO.LOW)
    p1.ChangeDutyCycle(100) # PWM 신호의 duty cycle을 50%로 설정
    print(encoderPosA)
    time.sleep(0.1)