/* 라이다 센서 연동 - 앞뒤좌우 회전 */ 

#include "wiringPi.h"               // analogRead(), pinMode(), delay() 함수 등 사용 
#include <softPwm.h>
#include <iostream>                 // C++ 입출력 라이브러리
#include <thread>
#include <chrono>
#include <ctime>
#include <string>
#include <unistd.h>
#include <algorithm>
#include <math.h>

#define M_PI 3.14159265358979323846
using namespace std;

/* 핀 번호가 아니라 wiringPi 번호 ! ----------------------------> 핀 설정 다시하기 
   DC 모터 왼쪽 (엔코더 O) */                                                     
#define pwmPinA 23      // 모터드라이버 ENA / ex) 핀 번호 8번, GPIO 14번, wiringPi 15번
#define AIN1 22         // IN1 
#define AIN2 21          // IN2 
#define encPinA 8       // 보라색 (A) 
#define encPinB 9       // 파랑색 (B) 

/* DC모터 오른쪽 (엔코더 X) */
#define pwmPinB 29       // 모터 드라이버 ENB 
#define BIN3 28          // IN3
#define BIN4 27          // IN4
#define encPinC 28      // 보라색 (C) - 20
#define encPinD 29      // 파랑색 (D) - 21

/* PID 제어 */
const float proportion = 360. / 264. / 52.;       // 한 바퀴에 약 13,728펄스 (정확하지 않음 - 계산값)

/* PID 상수 */
float kp = 30.0; 
float kd = 0.;         
float ki = 0.;

float encoderPosRight = 0;             // 엔코더 값 - 오른쪽
float encoderPosLeft = 0;              // 엔코더 값 - 왼쪽

float motorDegA = 0;                   // 모터 각도A
float motorDegB = 0;                   // 모터 각도B
float motor_distanceA;                 // 모터 거리 
float motor_distanceB;                 // 모터 거리 

float errorA = 0;
float errorB = 0;
float error_prev_A = 0.;
float error_prev_B = 0.;
float error_prev_prev_A = 0;
float error_prev_prev_B = 0;
float derrorA;
float derrorB;

double controlA = 0.;
double controlB = 0.;

double wheel; 
double target_deg;                      // 목표 각도 
double target_direction = 0.;           // 목표 방향 
double target_distance = 0.;            // 목표 거리 

double de_A;
double de_B;
double di_A = 0;
double di_B = 0;
double dt = 0;

double delta_vA;
double delta_vB;
double time_prev = 0;

double left_wheel_deg = 0;               // 왼쪽 바퀴 회전 각도 
double right_wheel_deg = 0;              // 오른쪽 바퀴 회전 각도
double turn_deg = 0;                     // 모터가 회전한 각도
double target_turn_deg;                  // 원하는 회전한 각도   
double wheelbase = 59;                   // 바퀴 사이 거리
double radius;                           // 회전 반지름
double deltaEnc;                         // 엔코더 값의 차이 
double carDistance;
double carMoving_filter;

int encoderPos_resolution = 52;          // 엔코더 해상도
int encoderPos_PR= 26;                  // 엔코더 분해능   
int frequency = 1024;                    // PWM 주파수 
int lidar_way;
int x;

std::time_t start_time = std::time(nullptr);

class MotorControl{
public:
    void call(int x);
    int getInput();
};

// 원하는 방향 입력
int MotorControl::getInput() {
    int x;
    cout << "직진 : 1 / 후진 : 2 / 오른쪽 : 3 / 왼쪽 : 4";
    cout << "원하는 방향을 입력하시오 : ";
    cin >> x;
    return x; 
}

void MotorControl::call(int x){
    // 정지
    if (x == 0){
        // 방향 설정 
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN3, LOW);
        digitalWrite(BIN4, LOW);
        delay(10);
        // 속도 설정 
        softPwmWrite(pwmPinA, 0);
        softPwmWrite(pwmPinB, 0);    
        
        // x(방향)의 값이 0(정지)이 아닐 경우 x(방향)을 다시 입력 받음 
        if(x != 0){
            x = getInput();
        }
    }
    
    // 전진
    else if (x == 1) {
        // 방향 설정 
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN3, LOW);
        digitalWrite(BIN4, HIGH);
        delay(10);
        // 속도 설정 
        softPwmWrite(pwmPinA, std::min(abs(controlA), 255.0));
        softPwmWrite(pwmPinB, std::min(abs(controlA), 255.0));   

        // x(방향)의 값이 1(전진)이 아닐 경우 x(방향)을 다시 입력 받음 
        if(x != 1){
            x = getInput();
        }
    }

    // 후진
    else if (x == 2) {
        // 방향 설정 
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN3, HIGH);
        digitalWrite(BIN4, LOW);   
        delay(10);
        // 속도 설정 
        softPwmWrite(pwmPinA, std::min(abs(controlA), 255.0));
        softPwmWrite(pwmPinB, std::min(abs(controlA), 255.0));   

        // x(방향)의 값이 2(후진)이 아닐 경우 x(방향)을 다시 입력 받음 
        if(x != 2){
            x = getInput();
        }   
    }

    // 오른쪽
    else if (x == 3){
        // 목표 각도 입력
        cout << "원하는 각도를 입력하시오 : ";
        cin >> target_turn_deg;
        
        // 방향 설정 
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN3, HIGH);
        digitalWrite(BIN4, LOW);
        delay(10);
        // 속도 설정 
        pwmWrite(pwmPinA, 50);
        pwmWrite(pwmPinB, 255);
       
        // x(방향)의 값이 3(오른쪽)이 아닐 경우 x(방향)을 다시 입력 받음 
        if(x != 3){
            x = getInput();
        }         
    }
            
    
    // 왼쪽
    else if (x == 4){
        // 목표 각도 입력 
        cout << "원하는 각도를 입력하시오 : ";
        cin >> target_turn_deg;        
        

        // 방향 설정 
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN3, LOW);
        digitalWrite(BIN4, HIGH);
        delay(10);
        // 속도 설정
        analogWrite(pwmPinA, min(abs(controlA), 255.0));
        analogWrite(pwmPinB, min(abs(controlA), 255.0));

        // x(방향)의 값이 4(왼쪽)이 아닐 경우 x(방향)을 다시 입력 받음        
        if(x != 4){
            x = getInput();
        }      
    }
}

int main(){
    wiringPiSetup();

    MotorControl control;

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

    while(true) {
   
        int lidar_way = control.getInput();
        control.call(lidar_way);
        delay(1000);
    
    }
    return 0;
}