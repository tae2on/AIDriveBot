/* 라이다 센서 연동 */ 

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <wiringPi.h>               // analogRead(), pinMode(), delay() 함수 등 사용 
#include <iostream>                 // C++ 입출력 라이브러리
#include <thread>
#include <chrono>

#define M_PI 3.14159265358979323846

using namespace std;



/* 핀 번호가 아니라 wiringPi 번호 ! ----------------------------> 핀 설정 다시하기 
   DC 모터 왼쪽 (엔코더 O) */                                                     
#define pwmPinA 15      // 모터드라이버 ENA / ex) 핀 번호 8번, GPIO 14번, wiringPi 15번
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

/* PID 제어 */
const float ratio = 360. / 264. / 52.;       // 한 바퀴에 약 13,728펄스 (정확하지 않음 - 계산값)

float encoderPosRight = 0;
float encoderPosLeft = 0;

float motorDegA = 0;
float motorDegB = 0;
float motor_distanceA = 0;
float motor_distanceB = 0;

float errorA = 0;
float errorB = 0;
float derrorA = 0;
float derrorB = 0;

double controlA = 0.;
double controlB = 0.;

double wheel; 
double target_deg;                      // 목표 각도 
double target_direction = 0.;           // 목표 방향 
double target_distance = 0.;            // 목표 거리 

void call(string vector);

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

/* 전진 */
void goFront() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN3, LOW);
    digitalWrite(BIN4, HIGH);
    delay(10);
    analogWrite(pwmPinA, min(abs(controlA), 255.0));
    analogWrite(pwmPinB, min(abs(controlA), 255.0));

    cout << "각도 = " << motorDegB << endl;
    cout << "ctrlA = " << controlA << ", degA = " << motorDegA << ", errA = " << errorA << ", disA = " << motor_distanceA << ", derrA = " << derrorA << endl;
    cout << "ctrlB = " << controlB << ", degB = " << motorDegB << ", errB = " << errorB << ", disB = " << motor_distanceB << ", derrB = " << derrorB << endl;

    return call(string vector);
}

/* 후진 */
void goBack() {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN3, HIGH);
    digitalWrite(BIN4, LOW);
    delay(10);
    analogWrite(pwmPinA, min(abs(controlA), 255.0));
    analogWrite(pwmPinB, min(abs(controlA), 255.0));

    cout << "각도 = " << motorDegB << endl;
    cout << "ctrlA = " << controlA << ", degA = " << motorDegA << ", errA = " << errorA << ", disA = " << motor_distanceA << ", derrA = " << derrorA << endl;
    cout << "ctrlB = " << controlB << ", degB = " << motorDegB << ", errB = " << errorB << ", disB = " << motor_distanceB << ", derrB = " << derrorB << endl;
   
    return call(string vector);
}

/* 정지 */
void Stop() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN3, LOW);
    digitalWrite(BIN4, LOW);
    delay(10);
    pwmWrite(pwmPinA, 0);
    pwmWrite(pwmPinB, 0);

    cout << "각도 = " << motorDegB << endl;
    cout << "ctrlA = " << controlA << ", degA = " << motorDegA << ", errA = " << errorA << ", disA = " << motor_distanceA << ", derrA = " << derrorA << endl;
    cout << "ctrlB = " << controlB << ", degB = " << motorDegB << ", errB = " << errorB << ", disB = " << motor_distanceB << ", derrB = " << derrorB << endl;

    return call(string vector);
}

/* 방향 설정하기 */
// 변수명 수정하기 
void call(string vector) {
    // 전진
    if (vector == "goFront") {
        goFront();
    }
    // 후진
    else if (vector == "goBack") {
        goBack();
    }
    // 정지 
    else if (vector == "Stop"){
        Stop();
    }
}


int main(){
    wiringPiSetup();

    pinMode(encPinA, INPUT);
    pullUpDnControl(encPinA, PUD_UP);
    pinMode(encPinB, INPUT);
    pullUpDnControl(encPinB, PUD_UP);
    pinMode(encPinC, INPUT);
    pullUpDnControl(encPinC, PUD_UP);
    pinMode(encPinD, INPUT);
    pullUpDnControl(encPinD, PUD_UP);
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

    wiringPiISR(encPinA, INT_EDGE_BOTH, &doEncoderA);
    wiringPiISR(encPinB, INT_EDGE_BOTH, &doEncoderB);
    wiringPiISR(encPinC, INT_EDGE_BOTH, &doEncoderC);
    wiringPiISR(encPinD, INT_EDGE_BOTH, &doEncoderD);

    while(true) {
        wheel = 2*M_PI*11.5;
        target_deg = (360*target_distance / wheel) ;      // 목표 각도
        

        call("goFront");
        delay(1000);
        call("goBack");
        delay(1000);
        call("Stop");
        delay(1000);
    }
}