/* 거리 제어 */

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

/* 핀 번호가 아니라 wiringPi 번호 ! */
/* gpio readall -> GPIO핀 / wiringPi핀 번호 확인법 */
/* ex) 핀 번호 8번, GPIO 14번, wiringPi 15번 */

// DC모터 오른쪽 (엔코더 X) 
#define pwmPinB 0              // 모터 드라이버 ENB - GPIO핀 번호 : 17
#define BIN3 15                  // IN3 - GPIO핀 번호 : 14
#define BIN4 9                 // IN4 - GPIO핀 번호 : 3
#define encPinC 24               // 보라색 (C) - GPIO핀 번호 : 19
#define encPinD 25               // 파랑색 (D) - GPIO핀 번호 : 26

/* PID 제어 */
const float proportion = 360. / (84 * 20);       // 한 바퀴에 약 1350펄스 (정확하지 않음 - 계산값)

/* PID 상수 */
float kp_B; 
float kd_B;         
float ki_B;

float encoderPosRight = 0.;            // 엔코더 값 - 오른쪽

float motorDegB = 0;                   // 모터 각도B

float derrorB = 0.;
float errorB = 0;
float error_prev_B = 0.;
float error_prev_prev_B = 0;

double turn_deg;                         // 회전 각도 
double target_deg = 360;                 // 목표 회전각도 
double controlB = 0.;

// 모터 이동거리 구할 때 필요
double target_distance;            // 목표 거리     

double de_B = 0;
double di_B = 0;
double dt = 0;
double dt_sleep = 0.01;

double delta_vB = 0;
double time_prev = 0;

std::time_t start_time = std::time(nullptr);

// 인터럽트 
void doEncoderC() {
  encoderPosRight  += (digitalRead(encPinC) == digitalRead(encPinD)) ? 1 : -1;
}
void doEncoderD() {
  encoderPosRight  += (digitalRead(encPinC) == digitalRead(encPinD)) ? -1 : 1;
}
   
void zero(){
    if (encoderPosRight != 0) {
        encoderPosRight = 0;
    }    
}
int main(){
    wiringPiSetup();

    pinMode(encPinC, INPUT);
    pullUpDnControl(encPinC, PUD_UP);
    pinMode(encPinD, INPUT);
    pullUpDnControl(encPinD, PUD_UP);
    pinMode(pwmPinB, OUTPUT); 
    pinMode(BIN3, OUTPUT);
    pinMode(BIN4, OUTPUT);

    softPwmCreate(pwmPinB, 0, 100);
    softPwmWrite(pwmPinB, 0); 

    digitalWrite(pwmPinB, LOW);
    digitalWrite(BIN3, LOW);
    digitalWrite(BIN4, LOW);

    wiringPiISR(encPinC, INT_EDGE_BOTH, &doEncoderC);
    wiringPiISR(encPinD, INT_EDGE_BOTH, &doEncoderD);   

    cout << "kp_B의 값 : ";
    cin >> kp_B;
    cout << "ki_B의 값 : ";
    cin >> ki_B;
    cout << "kd_B의 값 : ";
    cin >> kd_B;


    zero();    

    cout << "각도 = " << motorDegB << endl;cout << "ctrlB = " << controlB << ", degB = " << motorDegB << ", errB = " << errorB << endl;
    cout << "encB = " << encoderPosRight << endl;
    cout << "회전 각도 = " << turn_deg << endl;

    while (true){
        // DC모터 오른쪽
        motorDegB = abs(encoderPosRight * proportion);
        errorB = target_deg - motorDegB;
        de_B = errorB -error_prev_B;
        di_B += errorB * dt;
        dt = time(nullptr) - time_prev;
            
        delta_vB = kp_B*de_B + ki_B*errorB + kd_B*(errorB - 2*error_prev_B + error_prev_prev_B);
        controlB += delta_vB;
        error_prev_B = errorB;
        error_prev_prev_B = error_prev_B;
        
        // 방향 설정  
        digitalWrite(BIN3, HIGH);
        digitalWrite(BIN4, LOW); 

        delay(10);
        // 속도 설정 
        softPwmWrite(pwmPinB, min(abs(controlB), 100.));  

        // analogWrite(pwmPinA, min(abs(controlA), 0.0));
        //analogWrite(pwmPinB, min(abs(controlB), 100.0));

        cout << "----------------------------------------------------------------------------------" << endl;
        cout << "ctrlB = " << controlB << ", degB = " << motorDegB << ", errB = " << errorB << ", disB = " << endl;
        cout << "encB = " << encoderPosRight << endl;
        cout << "회전 각도 = " << turn_deg << endl;
            
        if (motor_distance_A >= target_distance){
            digitalWrite(BIN3, LOW);
            digitalWrite(BIN4, LOW);            
            delay(10);
            // 속도 설정 
            softPwmWrite(pwmPinB, 0);   
        }

        if (motor_distance_B >= target_distance){
            digitalWrite(BIN3, LOW);
            digitalWrite(BIN4, LOW);            
            delay(10);
            // 속도 설정 
            softPwmWrite(pwmPinB, 0);   
        }        
    }
    return 0; 
}