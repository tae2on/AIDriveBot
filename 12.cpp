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
const float proportion = 360. / (84 * 10);       // 한 바퀴에 약 1350펄스 (정확하지 않음 - 계산값)

/* PID 상수 */
float kp; 
float kd;         
float ki;

float encoderPosRight = 0;             // 엔코더 값 - 오른쪽
float encoderPosLeft = 0;              // 엔코더 값 - 왼쪽

float motorDegA = 0;                   // 모터 각도A
float motorDegB = 0;                   // 모터 각도B
float motor_distance_A = 0;           // 왼쪽 모터 이동거리 
float motor_distance_B = 0;           // 오른쪽 모터 이동거리  

float derrorA = 0.;
float derrorB = 0.;
float errorA = 0;
float errorB = 0;
float error_prev_A = 0.;
float error_prev_B = 0.;
float error_prev_prev_A = 0;
float error_prev_prev_B = 0;

//float delta_encoder;
float turn_deg;                         // 회전 각도 
/* // 회전각도 오차값
float turn_error;    
float target_turn_deg;                   
*/
double controlA = 0.;
double controlB = 0.;
 
double wheel; 
double target_deg;                      // 목표 각도 

// 모터 이동거리 구할 때 필요
double target_distance = 30.;            // 목표 거리     

double de_A = 0;
double de_B = 0;
double di_A = 0;
double di_B = 0;
double dt = 0;
double dt_sleep = 0.01;

double delta_vA = 0;
double delta_vB = 0;
double time_prev = 0;

std::time_t start_time = std::time(nullptr);

// 인터럽트 
void doEncoderA() {
  encoderPosLeft  += (digitalRead(encPinA) == digitalRead(encPinB)) ? -1 : 1;
}
void doEncoderB() {
  encoderPosLeft  += (digitalRead(encPinA) == digitalRead(encPinB)) ? 1 : -1;
}
void doEncoderC() {
  encoderPosRight  += (digitalRead(encPinC) == digitalRead(encPinD)) ? 1 : -1;
}
void doEncoderD() {
  encoderPosRight  += (digitalRead(encPinC) == digitalRead(encPinD)) ? -1 : 1;
}

void Calculation() {
    wheel = 2*M_PI*11.5;
    target_deg = (360*target_distance / wheel) ;      // 목표 각도
        
    //DC모터 왼쪽
    motorDegA = abs(encoderPosLeft * proportion);
    errorA = target_deg - motorDegA;
    de_A = errorA -error_prev_A;
    di_A += errorA * dt;
    dt = time(nullptr) - time_prev;
        
    delta_vA = kp*de_A + ki*errorA + kd*(errorA - 2*error_prev_A + error_prev_prev_A);
    controlA += delta_vA;
    error_prev_A = errorA;
    error_prev_prev_A = error_prev_A;
        
    motor_distance_A = motorDegA * wheel / 360;           // 모터 움직인 거리
    derrorA = abs(target_distance - motor_distance_A);    // 거리 오차값

     // DC모터 오른쪽
    motorDegB = abs(encoderPosRight * proportion);
    errorB = target_deg - motorDegB;
    de_B = errorB -error_prev_B;
    di_B += errorB * dt;
    dt = time(nullptr) - time_prev;
        
    delta_vB = kp*de_B + ki*errorB + kd*(errorB - 2*error_prev_B + error_prev_prev_B);
    controlB += delta_vB;
    error_prev_B = errorB;
    error_prev_prev_B = error_prev_B;

    motor_distance_B = motorDegB * wheel / 360;           // 모터 움직인 거리
    derrorB = abs(target_distance - motor_distance_B);    // 거리 오차값

    cout << "각도 = " << motorDegB << endl;
    cout << "ctrlA = " << controlA << ", degA = " << motorDegA << ", errA = " << errorA << ", disA = " << motor_distance_A << ", derrA = " << derrorA << endl;
    cout << "ctrlB = " << controlB << ", degB = " << motorDegB << ", errB = " << errorB << ", disB = " << motor_distance_B << ", derrB = " << derrorB << endl;
    cout << "encA = " << encoderPosLeft<< endl;
    cout << "encB = " << encoderPosRight << endl;
    cout << "회전 각도 = " << turn_deg << endl;
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

    Calculation(); 
    cout << "kp의 값 : ";
    cin >> kp;
    cout << "ki의 값 : ";
    cin >> ki;
    cout << "kd의 값 : ";
    cin >> kd;
    while (true){
        // 방향 설정  
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN3, HIGH);
        digitalWrite(BIN4, LOW); 

        delay(10);
        // 속도 설정 
        softPwmWrite(pwmPinA, 255.);
        softPwmWrite(pwmPinB, 255.);  

        cout << "각도 = " << motorDegB << endl;
        cout << "ctrlA = " << controlA << ", degA = " << motorDegA << ", errA = " << errorA << ", disA = " << motor_distance_A << ", derrA = " << derrorA << endl;
        cout << "ctrlB = " << controlB << ", degB = " << motorDegB << ", errB = " << errorB << ", disB = " << motor_distance_B << ", derrB = " << derrorB << endl;
        cout << "encA = " << encoderPosLeft<< endl;
        cout << "encB = " << encoderPosRight << endl;
        cout << "회전 각도 = " << turn_deg << endl;
            
        if (motor_distance_A >= target_deg) {
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN3, LOW);
            digitalWrite(BIN4, LOW);
            delay(10);
            // 속도 설정 
            softPwmWrite(pwmPinA, 0);
            softPwmWrite(pwmPinB, 0);    
        }
    }
    return 0;
}