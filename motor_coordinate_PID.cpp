/* 좌표를 이용한 PID값 */

#include "wiringPi.h"                   // analogRead(), pinMode(), delay() 함수 등 사용 
#include <softPwm.h>
#include <iostream>                    // C++ 입출력 라이브러리
#include <thread>
#include <chrono>
#include <ctime>
#include <string>
#include <unistd.h>
#include <algorithm>
#include <math.h>

#define M_PI 3.14159265358979323846
using namespace std;
using namespace std::chrono;

std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();  // 루프 시작 시간 기록

/* 핀 번호가 아니라 wiringPi 번호 ! */
/* gpio readall -> GPIO핀 / wiringPi핀 번호 확인법 */
/* ex) 핀 번호 8번, GPIO 14번, wiringPi 15번 */

// DC 모터 왼쪽 (엔코더 O)                                                      
#define pwmPinA 26             // 모터드라이버 ENA - GPIO핀 번호: 12
#define AIN1 27            // IN1 - GPIO핀 번호: 16
#define AIN2 6            // IN2 - GPIO핀 번호 : 25 
#define encPinA 4           // 보라색 (A) - GPIO핀 번호 : 23
#define encPinB 5           // 파랑색 (B) - GPIO핀 번호 : 24

// DC모터 오른쪽 (엔코더 X) 
#define pwmPinB 25           // 모터 드라이버 ENB - GPIO핀 번호 : 26    
#define BIN3 22           // IN3 - GPIO핀 번호 : 6
#define BIN4 21           // IN4 - GPIO핀 번호 : 5
#define encPinC 3            // 보라색 (C) - GPIO핀 번호 : 22
#define encPinD 0            // 파랑색 (D) - GPIO핀 번호 : 17

/* PID 제어 */
const float proportion = 360. / (84 * 4 * 10);       // 한 바퀴에 약 1350펄스 (정확하지 않음 - 계산값)

/* PID 상수*/
// 자리요 

volatile int encoderPosLeft = 0;              // 엔코더 값 - 왼쪽
volatile int encoderPosRight = 0;              // 엔코더 값 - 왼쪽 

double motorDegL = 0;
double motorDegR = 0;

double rad = M_PI / 180;
double deg = 180 / M_PI;

/* 원하는 x,y 좌표값, 각도값 */
double x_target_coordinate = 10;
double y_target_coordinate = 10;
double setha_target = 0;

/* 로봇의 선형 변위와 각변위 계산식 */
double delta_s = 0;
double delta_setha = 0;

/* 로봇의 위치, 방향각을 좌표로 계산식 */
double bar_setha = 0 ;
 
double x_coordinate = 0;
double x_prev_coordinate = 0;

double y_coordinate = 0;
double y_prev_coordinate = 0;

double setha_coordinate = 0;
double setha_prev_coordinate = 0;

/* 좌표 추적 계산식 */
double e1 = 0;
double e2 = 0;
double e3 = 0;

double vd = 0;
double vr = 1;  // 식 받기 
double wd = 0;
double wr = 0.5;

double k1 = 8.5203; // 양수 Ok  
double k2 = 116.7365;
double k3 = 0.1840;

/* 좌표값을 거리값 계산식 */
double 


// 인터럽트 
void doEncoderA() {
  encoderPosLeft  += (digitalRead(encPinA) == digitalRead(encPinB)) ? 1 : -1;
}
void doEncoderB() {
  encoderPosLeft  += (digitalRead(encPinA) == digitalRead(encPinB)) ? -1 : 1;
}
void doEncoderC() {
  encoderPosRight += (digitalRead(encPinC) == digitalRead(encPinD)) ? 1 : -1;
}
void doEncoderD() {
  encoderPosRight += (digitalRead(encPinC) == digitalRead(encPinD)) ? -1 : 1;
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
   
    softPwmCreate(pwmPinA, 0, 100);
    softPwmCreate(pwmPinB, 0, 100);
    softPwmWrite(pwmPinA, 0);
    softPwmWrite(pwmPinB, 0); 

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

    while (true){
        motorDegL = encoderPosLeft * proportion * rad;
        motorDegR = encoderPosRight * proportion * rad;

        /* 로봇의 선형 변위와 각변위 계산식 */
        delta_s = 11.5 / 2 * (motorDegL + motorDegR);
        delta_setha = 11.5 / 29.2 * (motorDegR - motorDegL);

        /* 로봇의 위치와 방향각 계산식 */
        bar_setha = setha_prev_coordinate + delta_setha / 2;
        
        // DC모터 x좌표 
        x_coordinate = x_prev_coordinate + cos(bar_setha) * delta_s;
        x_prev_coordinate = x_coordinate;

        // DC모터 y좌표
        y_coordinate = y_prev_coordinate + sin(bar_setha) * delta_s;
        y_prev_coordinate = y_coordinate;
        
        // DC모터 방향각
        setha_coordinate = setha_prev_coordinate + delta_setha;
        setha_prev_coordinate = setha_coordinate;

        /* 좌표 추적 계산식 */
        e1 = cos(setha_coordinate) * (x_target_coordinate - x_coordinate) + sin(setha_coordinate) * (y_target_coordinate - y_coordinate);
        e2 = -sin(setha_coordinate) * (x_target_coordinate - x_coordinate) + cos(setha_coordinate) * (y_target_coordinate - y_coordinate);
        e3 = setha_target - setha_coordinate;

        vd = vr * cos(e3) + k1 * e1;
        wd = wr + k2 * vr * e2 * (sin(e3) / e3) + k2 * k3 * e3;

        /* 좌표값을 거리값으로 계산식*/


        /* 좌표값 PID 계산 */
        // 오차 계산



    }
  return 0; 
}