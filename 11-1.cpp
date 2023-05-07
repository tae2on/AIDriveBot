/* 라이다 센서 연동 */ 

#include "wiringPi.h"               // analogRead(), pinMode(), delay() 함수 등 사용 
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
// DC 모터 왼쪽 (엔코더 O)                                                      
#define pwmPinA 21              // 모터드라이버 ENA - GPIO핀 번호: 5
#define AIN1 27                 // IN1 - GPIO핀 번호: 16
#define AIN2 28                 // IN2 - GPIO핀 번호 : 20
#define encPinA 22               // 보라색 (A) - GPIO핀 번호 : 6
#define encPinB 23               // 파랑색 (B) - GPIO핀 번호 : 13

// DC모터 오른쪽 (엔코더 X) 
#define pwmPinB 0              // 모터 드라이버 ENB - GPIO핀 번호 : 17
#define BIN3 15                  // IN3 - GPIO핀 번호 : 14
#define BIN4 9                 // IN4 - GPIO핀 번호 : 3
#define encPinC 24               // 보라색 (C) - GPIO핀 번호 : 19
#define encPinD 25               // 파랑색 (D) - GPIO핀 번호 : 26

/* PID 제어 */
const float proportion = 360. / 144. / 10.;       // 한 바퀴에 약 1350펄스 (정확하지 않음 - 계산값)

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


/* 인터럽트 */
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
    void goFront();
    void goBack();
    void goRight();
    void goLeft();
    void Stop();
};

void Calculation() {
        wheel = 2*M_PI*11.5;
        target_deg = (360*target_distance / wheel) ;      // 목표 각도
        
        /* DC모터 왼쪽 */
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

        /* DC모터 오른쪽 */
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
        
        // 두 엔코더 값의 차이
        deltaEnc = encoderPosRight - encoderPosLeft;

        // 자동차가 이동한 거리 
        carDistance = deltaEnc * (M_PI * 23.0 / encoderPos_resolution);

        // 자동차가 회전한 각도
        turn_deg = carDistance / wheelbase;

        // 보정 계수
        carMoving_filter = 0.44;     //0.4<x<0.5  
        turn_deg *= carMoving_filter;

        cout << "각도 = " << motorDegA << endl;
        cout << "ctrlA = " << controlA << ", degA = " << motorDegA << ", errA = " << errorA << ", disA = " << motor_distanceA << ", derrA = " << derrorA << endl;
        cout << "ctrlB = " << controlB << ", degB = " << motorDegB << ", errB = " << errorB << ", disB = " << motor_distanceB << ", derrB = " << derrorB << endl;
        cout << "encA = " << encoderPosLeft<< endl;
        cout << "encB = " << encoderPosRight << endl;
        cout << "회전 각도 = " << turn_deg << endl;
} 

void MotorControl::call(int x){
    // 정지
    if (x == 0){
        Stop();
    }
    // 전진
    else if (x == 1) {
        goFront();
    }
    // 후진
    else if (x == 2) {
        goBack();     
    }
    // 오른쪽
    else if (x == 3){
        cout << "원하는 각도를 입력하시오 : ";
        cin >> target_turn_deg;
        goRight();
    }
    // 왼쪽
    else if (x == 4){
        cout << "원하는 각도를 입력하시오 : ";
        cin >> target_turn_deg;        
        goLeft();
    }
}

/* 전진 */
void MotorControl::goFront() {
    while(true) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN3, LOW);
        digitalWrite(BIN4, HIGH);
        delay(10);
        analogWrite(pwmPinA, std::min(abs(controlA), 20.0));
        analogWrite(pwmPinB, std::min(abs(controlB), 255.0));

        Calculation();

        if(x != 1){
            break;
        }
    }
}

/* 후진 */
void MotorControl::goBack() {
    while(true) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN3, HIGH);
        digitalWrite(BIN4, LOW);
        delay(10);
        pwmWrite(pwmPinA, 3);
        pwmWrite(pwmPinB, 255);       
        //analogWrite(pwmPinA, min(abs(controlA), 255.0));
        //analogWrite(pwmPinB, min(abs(controlA), 255.0));

        Calculation();
   
        if(x != 2){
            break;
        }        
    }
}

/* 오른쪽 */
void MotorControl::goRight() {
    while(true) {            
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN3, HIGH);
        digitalWrite(BIN4, LOW);
        delay(10);
        analogWrite(pwmPinA, min(abs(controlA), 255.0));
        analogWrite(pwmPinB, min(abs(controlA), 255.0));

        Calculation();
        
        if ((turn_deg >= target_turn_deg)){
            break;
        }       
    }
    Stop();        
}


/* 왼쪽 */
void MotorControl::goLeft() {
    while(true) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN3, LOW);
        digitalWrite(BIN4, HIGH);
        delay(10);
        analogWrite(pwmPinA, min(abs(controlA), 255.0));
        analogWrite(pwmPinB, min(abs(controlA), 255.0));

        Calculation();
        
        if ((turn_deg >= target_turn_deg)){
            break;
        }       
    }
    Stop();
}

/* 정지 */
void MotorControl::Stop() {
    while(true) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN3, LOW);
        digitalWrite(BIN4, LOW);
        delay(10);
        pwmWrite(pwmPinA, 0);
        pwmWrite(pwmPinB, 0);

        Calculation();

        if(x != 0){
            break;
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
    pinMode(pwmPinA, PWM_OUTPUT); // PWM 출력으로 사용할 핀을 설정합니다.
    pinMode(pwmPinB, PWM_OUTPUT); // PWM 출력으로 사용할 핀을 설정합니다.
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

    pwmSetRange(frequency);     // PWM 값의 범위를 설정합니다.

    pwmWrite(pwmPinA, 0);       // PWM 신호의 듀티 사이클을 0으로 설정합니다.
    pwmWrite(pwmPinB, 0);       // PWM 신호의 듀티 사이클을 0으로 설정합니다.

    wiringPiISR(encPinA, INT_EDGE_BOTH, &doEncoderA);
    wiringPiISR(encPinB, INT_EDGE_BOTH, &doEncoderB);
    wiringPiISR(encPinC, INT_EDGE_BOTH, &doEncoderC);
    wiringPiISR(encPinD, INT_EDGE_BOTH, &doEncoderD);


    while(true) {

        int lidar_way;
        cout << "값을 입력하시오 : ";
        cin >> lidar_way; 
        
        control.call(lidar_way);
        delay(1000);
    }
}