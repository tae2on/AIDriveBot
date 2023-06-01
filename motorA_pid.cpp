/* 각도 PID 제어 */

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

/* PID 제어 */
const float proportion = 360. / (84 * 4 * 10);       // 한 바퀴에 약 1350펄스 (정확하지 않음 - 계산값)

/* PID 상수 */
float kp_A; 
float kd_A;         
float ki_A;

volatile int encoderPosLeft = 0;              // 엔코더 값 - 왼쪽

float motorDegA = 0;                   // 모터 각도A

float errorA = 0;
float error_prev_A = 0.;
float error_prev_prev_A = 0.;

double target_deg = 360;                 // 목표 회전각도 
double controlA = 0.;
double delta_vA = 0;

// 인터럽트 
void doEncoderA() {
  encoderPosLeft  += (digitalRead(encPinA) == digitalRead(encPinB)) ? -1 : 1;
}
void doEncoderB() {
  encoderPosLeft  += (digitalRead(encPinA) == digitalRead(encPinB)) ? 1 : -1;
}
   
int main(){
    wiringPiSetup();

    pinMode(encPinA, INPUT);
    pullUpDnControl(encPinA, PUD_UP);
    pinMode(encPinB, INPUT);
    pullUpDnControl(encPinB, PUD_UP);
    pinMode(pwmPinA, OUTPUT); 
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
   
    softPwmCreate(pwmPinA, 0, 100);
    softPwmWrite(pwmPinA, 0);

    digitalWrite(pwmPinA, LOW);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);

    wiringPiISR(encPinA, INT_EDGE_BOTH, &doEncoderA);
    wiringPiISR(encPinB, INT_EDGE_BOTH, &doEncoderB);

    cout << "kp_A의 값 : ";
    cin >> kp_A;
    cout << "ki_A의 값 : ";
    cin >> ki_A;
    cout << "kd_A의 값 : ";
    cin >> kd_A;   

    cout << "각도 = " << motorDegA << endl;
    cout << "ctrlA = " << controlA << ", degA = " << motorDegA << endl;
    cout << "encA = " << encoderPosLeft<< endl;
    cout << "회전 각도 = " << turn_deg << endl;
    cout << "errorA = " << errorA << ", error_prev_A = " << error_prev_A << ", error_prev_prev_A = " << error_prev_prev_A << endl;

    while (true){
        //DC모터 왼쪽
        motorDegA = abs(encoderPosLeft * proportion);

        errorA = target_deg - motorDegA;
        delta_vA = kp_A * (errorA - error_prev_A) + ki_A * errorA + kd_A * (errorA - 2 * error_prev_A + error_prev_prev_A);
        controlA += delta_vA;
        
        // 방향 설정  
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // 속도 설정 
        softPwmWrite(pwmPinA, min(abs(controlA), 100.));    

        // analogWrite(pwmPinA, min(abs(controlA), 0.0));
        //analogWrite(pwmPinB, min(abs(controlB), 100.0));

        cout << "--------------------------------------------------------------------------------" << endl;
         cout << "각도 = " << motorDegA << endl;
        cout << "ctrlA = " << controlA << ", degA = " << motorDegA << endl;
        cout << "encA = " << encoderPosLeft<< endl;
        cout << "회전 각도 = " << turn_deg << endl;
        cout << "errorA = " << errorA << ", error_prev_A = " << error_prev_A << ", error_prev_prev_A = " << error_prev_prev_A << endl;
            
        if (motorDegA >= target_deg){
            softPwmWrite(pwmPinA, 0); 
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, LOW);       
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            // 속도 설정 
            
            controlA = 0;
        }
    }    
    return 0; 
}