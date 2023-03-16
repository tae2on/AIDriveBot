/* 라이다 센서 연동 */ 

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <wiringPi.h>               // analogRead(), pinMode(), delay() 함수 등 사용 

/* 핀 번호가 아니라 GPIO 번호 ! ----------------------------> 핀 설정 다시하기 
   DC 모터 왼쪽 (엔코더 O) */                                                     
#define pwmPinA 15      // 모터드라이버 ENA / ex) 핀 번호 8번, GPIO 14번
#define AIN1 16         // IN1 
#define AIN2 1          // IN2 
#define encPinA 8       // 보라색 (A) 
#define encPinB 9       // 파랑색 (B) 

/* DC모터 오른쪽 (엔코더 X) */
#define pwmPinB 0       // 모터 드라이버 ENB 
#define BIN3 2          // IN3
#define BIN4 3          // IN4
#define encPinC 28      // 보라색 (C) - 20
#define encPinD 29      // 파랑색 (D) - 21

float encoderPosRight = 0;
float encoderPosLeft = 0;

void doEncoderA() {
  encoderPosRight  += (digitalRead(encPinA) == digitalRead(encPinB)) ? 1 : -1;
}
void doEncoderB() {
  encoderPosRight  += (digitalRead(encPinA) == digitalRead(encPinB)) ? -1 : 1;
}
void doEncoderC() {
  encoderPosLeft  += (digitalRead(encPinC) == digitalRead(encPinD)) ? 1 : -1;
}
void doEncoderD() {
  encoderPosLeft  += (digitalRead(encPinC) == digitalRead(encPinD)) ? -1 : 1;
}

int main(){
    pinMode(encPinA, INPUT_PULLUP);
    pinMode(encPinB, INPUT_PULLUP);
    pinMode(encPinC, INPUT_PULLUP);
    pinMode(encPinD, INPUT_PULLUP);
    pinMode(pwmPinA, OUTPUT);
    pinMode(pwmPinB, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN3, OUTPUT);
    pinMode(BIN4, OUTPUT);
   
    digitalWrite(pwmPinA, LOW);
    digitalWrite(pwmPinB, LOW);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN3, LOW);
    digitalWrite(BIN4, LOW);


    wiringPiSetup();

    wiringPiISR(encPinA, INT_EDGE_BOTH, &doEncoderA);
    wiringPiISR(encPinB, INT_EDGE_BOTH, &doEncoderB);
    wiringPiISR(encPinC, INT_EDGE_BOTH, &doEncoderC);
    wiringPiISR(encPinD, INT_EDGE_BOTH, &doEncoderD);
}