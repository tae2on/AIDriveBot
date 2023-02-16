# 모터 속도 측정하기 (RPM, deg/s) 

import RPi.GPIO as IO
import time

IO.setmode(IO.BOARD)

AIN1= 13
AIN2= 15   
PWMA= 12

# 듀티 비를 변화시킬 스텝 정의 
c_step = 10   

#각 핀을 출력 핀으로 설정 
IO.setup(AIN1, IO.OUT, initial=IO.LOW)
IO.setup(AIN2, IO.OUT, initial=IO.LOW)
IO.setup(PWMA, IO.OUT, initial=IO.LOW)

#PWM 객체 인스턴스 작성(출력 핀:12, 주파수 100)
p = IO.PWM(PWMA, 100)
p.start(0)

try:
	while 1:
		IO.output(AIN1, IO.HIGH)
        # IO.output(AIN2, IO.LOW)
        p.ChangeDutyCycle(100)

except KeyboardInterrupt: 
	pass 

p.stop() 

IO.cleanup()