/* 각도 45도 PID 제어 */

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

/* PID 상수 */
float kp_L = 0.5; // 0.5
float kd_L = 0; // 0        
float ki_L = 0; // 0 

volatile int encoderPosLeft = 0;              // 엔코더 값 - 왼쪽
 
float motorDegL = 0;                   // 모터 각도A

float errorL = 0;
double error_tL = 0;
float error_prev_L = 0.;
float error_prev_prev_L = 0.;

double target_deg = 360;                 // 목표 회전각도 
double trun_deg = 45;
double controlL = 0.;
double delta_vL = 0;

double delta_deg = 0;
double deg_coordinate = 0;
double deg_prev_coordinate = 0;

auto start = std::chrono::high_resolution_clock::now();

// 인터럽트 
void doEncoderA() {
  encoderPosLeft  += (digitalRead(encPinA) == digitalRead(encPinB)) ? 1 : -1;
}
void doEncoderB() {
  encoderPosLeft  += (digitalRead(encPinA) == digitalRead(encPinB)) ? -1 : 1;
}
   
int main(){
    wiringPiSetup();

    pinMode(encPinA, INPUT);
    pullUpDnControl(encPinA, PUD_UP);
    pinMode(encPinB, INPUT);
    pullUpDnControl(encPinB, PUD_UP);
    pinMode(pwmPinA, OUTPUT); 
    pinMode(pwmPinB, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN3, OUTPUT);
    pinMode(BIN4, OUTPUT);

    softPwmCreate(pwmPinA, 0, 100);
    softPwmWrite(pwmPinA, 0);
    softPwmCreate(pwmPinB, 0, 100);
    softPwmWrite(pwmPinB, 0);

    digitalWrite(pwmPinA, LOW);
    digitalWrite(pwmPinB, LOW);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN3, LOW);
    digitalWrite(BIN4, LOW);

    wiringPiISR(encPinA, INT_EDGE_BOTH, &doEncoderA);
    wiringPiISR(encPinB, INT_EDGE_BOTH, &doEncoderB); 

    while (true){
        // DC모터 왼쪽
        motorDegL = abs(encoderPosLeft * proportion);
        errorL = target_deg - motorDegL;
        error_tL = trun_deg - deg_coordinate;
        cout << "--------------------------------------------------------------------------------" << endl;
        cout << "각도 = " << motorDegL << endl;
        cout << "ctrlA = " << controlL << ", degA = " << motorDegL << endl;
        cout << "encA = " << encoderPosLeft<< endl;
        cout << "errorA = " << errorL << ", error_prev_A = " << error_prev_L << ", error_prev_prev_A = " << error_prev_prev_L << endl;
        cout << "delta_deg = " << delta_deg << "deg_coordinate = " << deg_coordinate <<  endl;
        cout << "작업 실행 시간: " << duration.count() << " ms" << endl;        // 시간 출력
 
        delta_vL = kp_L * (errorL - error_prev_L) + ki_L * errorL + kd_L * (errorL - 2 * error_prev_L + error_prev_prev_L);
        controlL += delta_vL;

        error_prev_prev_L = error_prev_L;
        error_prev_L = errorL;

        delta_deg = 11.5 / 29.2 * (motorDegL + motorDegL);

        deg_coordinate = deg_prev_coordinate + delta_deg;
        deg_prev_coordinate = deg_coordinate;
   
        // 시간 측정 시작
        auto start = high_resolution_clock::now();

        // 방향 설정 
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN3, HIGH);
        digitalWrite(BIN4, LOW);
        // 속도 설정 
        softPwmWrite(pwmPinA, min(abs(controlL), 50.));           // 만약에 동작 안 할 경우 255. -> 100. 으로 수정    
        softPwmWrite(pwmPinB, min(abs(controlL), 50.));     

        auto start = std::chrono::high_resolution_clock::now();  // 루프 시작 시간 기록
     
        if (deg_coordinate >= trun_deg){
            softPwmWrite(pwmPinA, 0); 
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, LOW);      
            
            // 시간 측정 종료
            auto end = high_resolution_clock::now();
            auto duration = duration_cast<milliseconds>(end - start);

            break;
        }
      
        auto end = std::chrono::high_resolution_clock::now();  // 루프 종료 시간 기록
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);  // 루프 실행 시간 계산
        std::this_thread::sleep_for(std::chrono::milliseconds(10) - duration);  // 루프 실행 시간이 10ms가 되도록 대기
    
    }    
  return 0; 
}