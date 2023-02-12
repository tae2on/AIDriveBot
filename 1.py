# DC모터 속도/방향 조절하기 
# 천천히 회전하기 시작해서 회전 속도를 서서히 올리다가 최고 속도가 되면 다시 천천히 회전 속도를 낮춰서 정지 (회전 방향 바꿈)


import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

AIN1= 13
AIN2= 15   
PWMA= 12

# 듀티 비를 변화시킬 스텝 정의 
c_step = 10   

#각 핀을 출력 핀으로 설정 
GPIO.setup(AIN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(AIN2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(PWMA, GPIO.OUT, initial=GPIO.LOW)

#PWM 객체 인스턴스 작성(출력 핀:12, 주파수 100)
p = GPIO.PWM(PWMA, 100)
p.start(0)

try:
	while 1:
		GPIO.output(AIN1, GPIO.HIGH)
		for pw in range(0, 101, c_step): 
			p.ChangeDutyCycle(pw)
			time.sleep(0.5)
		for pw in range(100, -1, c_step * -1): 
			p.ChangeDutyCycle(pw)
			time.sleep(0.5)
		GPIO.output(AIN1, GPIO.LOW)
		time.sleep(0.5)
		GPIO.output(AIN2, GPIO.HIGH)
		for pw in range(0, 101, c_step ): 
			p.ChangeDutyCycle(pw)
			time.sleep(0.5)

		for pw in range(100, -1, c_step * -1): 
			p.ChangeDutyCycle(pw) 
			time.sleep(0.5)
			GPIO.output(AIN2, GPIO.LOW)
			time.sleep(0.5)

except KeyboardInterrupt: 
	pass 

p.stop() 

GPIO.cleanup()