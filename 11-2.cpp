/* 라이다 센서 연동 */
/* 두 모터의 속도 차이를 이용한 전/후/좌/우 회전 */ 

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

// DC 모터 왼쪽 (엔코더 O)                                                      
#define pwmPinA 23              // 모터드라이버 ENA - GPIO핀 번호: 13 
#define AIN1 22                 // IN1 - GPIO핀 번호: 6
#define AIN2 21                 // IN2 - GPIO핀 번호 : 5
#define encPinA 8               // 보라색 (A) - GPIO핀 번호 : 2
#define encPinB 9               // 파랑색 (B) - GPIO핀 번호 : 3

// DC모터 오른쪽 (엔코더 X) 
#define pwmPinB 29              // 모터 드라이버 ENB - GPIO핀 번호 : 21
#define BIN3 7                  // IN3 - GPIO핀 번호 : 4
#define BIN4 27                 // IN4 - GPIO핀 번호 : 16
#define encPinC 2               // 보라색 (C) - GPIO핀 번호 : 27
#define encPinD 3               // 파랑색 (D) - GPIO핀 번호 : 22

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
float motor_distanceA = 0.;                 // 모터 거리 
float motor_distanceB = 0.;                 // 모터 거리 

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

int lidar_way;
int x;

std::time_t start_time = std::time(nullptr);

// 인터럽트 
void doEncoderA() {
  encoderPosLeft  += (digitalRead(encPinA) == digitalRead(encPinB)) ? 1 : -1;
}
void doEncoderB() {
  encoderPosLeft  += (digitalRead(encPinA) == digitalRead(encPinB)) ? -1 : 1;
}
void doEncoderC() {
  encoderPosRight  += (digitalRead(encPinC) == digitalRead(encPinD)) ? 1 : -1;
}
void doEncoderD() {
  encoderPosRight  += (digitalRead(encPinC) == digitalRead(encPinD)) ? -1 : 1;
}

class MotorControl{
public:
    void call(int x);
    int getInput();
};

void Calculation() {
    wheel = 2*M_PI*11.5;
    target_deg = (360*target_distance / wheel) ;      // 목표 각도
        
    //DC모터 왼쪽
    motorDegA = encoderPosLeft * proportion;
    errorA = target_deg - motorDegA;
    de_A = errorA -error_prev_A;
    di_A += errorA * dt;
    dt = time(nullptr) - time_prev;
        
    delta_vA = kp*de_A + ki*errorA + kd*(errorA - 2*error_prev_A + error_prev_prev_A);
    controlA += delta_vA;
    error_prev_A = errorA;
    error_prev_prev_A = error_prev_A;

    motor_distanceA = motorDegA * wheel / 360;           // 모터 움직인 거리
    derrorA = abs(target_distance - motor_distanceA);    // 거리 오차값

    //DC모터 오른쪽
    motorDegB = encoderPosRight * proportion;
    errorB = target_deg - motorDegB;
    de_B = errorB -error_prev_B;
    di_B += errorB * dt;
    dt = time(nullptr) - time_prev;
        
    delta_vB = kp*de_B + ki*errorB + kd*(errorB - 2*error_prev_B + error_prev_prev_B);
    controlB += delta_vB;
    error_prev_B = errorB;
    error_prev_prev_B = error_prev_B;

    motor_distanceB = motorDegB * wheel / 360;           // 모터 움직인 거리
    derrorB = abs(target_distance - motor_distanceB);    // 거리 오차값
        
}
// 원하는 방향 입력
int MotorControl::getInput() {
    int x;
    cout << "정지 : 0 / 직진 : 1 / 후진 : 2 / 오른쪽 : 3 / 왼쪽 : 4" << endl;
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
        softPwmWrite(pwmPinA, 100.);
        softPwmWrite(pwmPinB, 100.);

        cout << "왼쪽 모터 움직인 각도 = " << motor_distanceA << endl;
        cout << "오른쪽 모터 움직인 각도 = " << motor_distanceB << endl;
        cout << "왼쪽 모터의 오차값 = " << derrorA << endl;
        cout << "오른쪽 모터의 오차값 = " << derrorB << endl;
  
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
        softPwmWrite(pwmPinA, 100.);
        softPwmWrite(pwmPinB, 100.);   

        // x(방향)의 값이 2(후진)이 아닐 경우 x(방향)을 다시 입력 받음 
        if(x != 2){
            x = getInput();
        }   
    }

    // 오른쪽
    else if (x == 3){
        // 목표 각도 입력
        //cout << "원하는 각도를 입력하시오 : ";
        //cin >> target_turn_deg;
        
        // 방향 조절 
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN3, LOW);
        digitalWrite(BIN4, HIGH);
        delay(10);
        // 속도 설정 
        softPwmWrite(pwmPinA, 255);
        softPwmWrite(pwmPinB, 30);  
       
        // x(방향)의 값이 3(오른쪽)이 아닐 경우 x(방향)을 다시 입력 받음 
        if(x != 3){
            x = getInput();
        }         
    }
            
    
    // 왼쪽
    else if (x == 4){
        // 목표 각도 입력 
        //cout << "원하는 각도를 입력하시오 : ";
        //cin >> target_turn_deg;        
        
        // 방향 조절 
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN3, LOW);
        digitalWrite(BIN4, HIGH);
        delay(10);
        // 속도 설정 
        softPwmWrite(pwmPinA, 30);
        softPwmWrite(pwmPinB, 255);  

        // x(방향)의 값이 4(왼쪽)이 아닐 경우 x(방향)을 다시 입력 받음        
        if(x != 4){
            x = getInput();
        }      
    }
}

int main(){
    wiringPiSetup();

    MotorControl control;

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
   
    softPwmCreate(pwmPinA, 0, 255);
    softPwmCreate(pwmPinB, 0, 255);
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

    while(true) {
   
        int lidar_way = control.getInput();
        control.call(lidar_way);
        delay(1000);
    
    }
    return 0;
}