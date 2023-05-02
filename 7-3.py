import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

encPinA = 2
encPinB = 3

GPIO.setup(encPinA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encPinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)

encoderPosA = 0

def encoderA(channel):
    global encoderPosA
    if GPIO.input(encPinA) == GPIO.input(encPinB):
        encoderPosA += 1
    else:
        encoderPosA -= 1

def encoderB(channel):
    global encoderPosA
    if GPIO.input(encPinA) == GPIO.input(encPinB):
        encoderPosA -= 1
    else:
        encoderPosA += 1

GPIO.add_event_detect(encPinA, GPIO.BOTH, callback=encoderA)
GPIO.add_event_detect(encPinB, GPIO.BOTH, callback=encoderB)

while True:
    print(encoderPosA)
    time.sleep(0.1)