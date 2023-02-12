// 캡스톤 모터 PID 제어 (c++)

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <wiringPi.h>

// 오른쪽 모터 엔코더 위치 제어
#define encPinA 8 // 인터럽트에 쓸 수 있는 디지털 핀 - 파란색 2번
#define encPinB 9 // 인터럽트에 쓸 수 있는 디지털 핀 - 보라색 3번

#define AIn1 16 // IN1
#define AIn2 1 // IN2
#define pwmPinR  15

const float ratio = 360. / 264. / 52.;

float encoderPosRight = 0;
float motorDeg = 0;
float setha = 0;
float error = 0;
float Kp;
float control;

int in1, in2;
int pwm;
int setha1;

void doEncoderA() {
  encoderPosRight  += (digitalRead(encPinA) == digitalRead(encPinB)) ? 1 : -1;
}
void doEncoderB() {
  encoderPosRight  += (digitalRead(encPinA) == digitalRead(encPinB)) ? -1 : 1;
}


wiringPiISR(PhaseA, INT_EDGE_BOTH, &Interrupt_A);
wiringPiISR(PhaseB, INT_EDGE_BOTH, &Interrupt_B);

/*void interruptInit() {
  attachInterrupt(0, doEncoderA, CHANGE);   //uno pin 2
  attachInterrupt(1, doEncoderB, CHANGE);   //uno pin 3
}*/

void setha_serial();

void setup() {
  Serial.begin(115200);
  pinMode(pwmPinR, OUTPUT);
  pinMode(AIn1, OUTPUT);
  pinMode(AIn2, OUTPUT);
  pinMode(encPinA, INPUT_PULLUP);
  pinMode(encPinB, INPUT_PULLUP);
  interruptInit();
  digitalWrite(AIn1, LOW);
  digitalWrite(AIn2, LOW);
  analogWrite(pwmPinR, 0);

}

void loop() {
  Serial.print("setha : ");
  Serial.print(setha);
  Serial.print(" / right : ");
  Serial.print(encoderPosRight);

  motorDeg = encoderPosRight * ratio;
  error = setha - motorDeg;
  control = Kp * error;
  pwm = min(abs(control), 255);


  Serial.print(" / motorDeg : ");
  Serial.print(motorDeg);
  Serial.print(" / error : ");
  Serial.print(error);
  Serial.print(" / control : ");
  Serial.println(control);
  
  setha_serial();
}

void setha_serial() {
  printf("각도를 입력하시오 : ");
  scanf("%d", &setha);
  }
  if (setha < 0) {
    in1 = HIGH, in2 = LOW;

    if ((setha >= motorDeg) && (control >= 0)) {
      in1 = LOW, in2 = LOW;
      digitalWrite(AIn1, in1);
      digitalWrite(AIn2, in2);
      analogWrite(pwmPinR, 0);
    }
    Kp = 12; // DC Power supply 22.5
    digitalWrite(AIn1, LOW);
    digitalWrite(AIn2, LOW);
    delay(20);
    digitalWrite(AIn1, in1);
    digitalWrite(AIn2, in2);
    analogWrite(pwmPinR, pwm);
  }
  else if (setha > 0) {
    in1 = LOW, in2 = HIGH;

    if ((setha <= motorDeg) && (control <= 0)) {
      in1 = LOW, in2 = LOW;
      digitalWrite(AIn1, in1);
      digitalWrite(AIn2, in2);
      analogWrite(pwmPinR, 0);
    }
    Kp = 11; // DC Power supply 20
    digitalWrite(AIn1, LOW);
    digitalWrite(AIn2, LOW);
    delay(20);
    digitalWrite(AIn1, in1);
    digitalWrite(AIn2, in2);
    analogWrite(pwmPinR, pwm);
  }
  else if (setha == 0) {
    in1 = LOW, in2 = LOW;
    digitalWrite(AIn1, LOW);
    digitalWrite(AIn2, LOW);
    delay(20);
    digitalWrite(AIn1, in1);
    digitalWrite(AIn2, in2);
    analogWrite(pwmPinR, 0);

    encoderPosRight = 0;
    setha = 0;
    motorDeg = 0;
    error = 0;
  }