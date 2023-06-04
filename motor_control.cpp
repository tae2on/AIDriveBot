/* 라이다 센서 연동 */
/* 모터 제어 없음 */

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
float kp_L = 0.5;
float kd_L = 0;      
float ki_L = 0;

// R 값 실험을 통해 찾기 
float kp_R;
float kd_R;      
float ki_R;

int encoderPosLeft = 0;
int encoderPosRight = 0; 

float motorDegL = 0;                   // 모터 각도 L
float motorDegR = 0;                   // 모터 각도 R

float errorL = 0;
float error_prev_L = 0.;
float error_prev_prev_L = 0.;

float errorR = 0;
float error_prev_R = 0.;
float error_prev_prev_R = 0.;

double target_deg;                 // 목표 회전각도 -> 식 다시 생각해보기 
double controlL = 0.;
double delta_vL = 0;

double controlR = 0.;
double delta_vR = 0;

/* 로봇의 선형 변위와 각변위 계산식 */
// 선형 변위 
double delta_s = 0;

// 각변위 
double delta_deg = 0;

/* 로봇의 위치, 방향각을 좌표로 계산식 */
double setha = 0 ;
double setha_prev = 0;

double x_coordinate = 0;
double x_prev_coordinate = 0;

double y_coordinate = 0;
double y_prev_coordinate = 0;

double deg_coordinate = 0;
double deg_prev_coordinate = 0;

/* 라이다 연동 */
int lidar_way;
int x;

std::time_t start_time = std::time(nullptr);

// 인터럽트 
void doEncoderA() {
  encoderPosLeft  += (digitalRead(encPinA) == digitalRead(encPinB)) ? -1 : 1;
}
void doEncoderB() {
  encoderPosLeft  += (digitalRead(encPinA) == digitalRead(encPinB)) ? 1 : -1;
}
void doEncoderC() {
  encoderPosRight += (digitalRead(encPinC) == digitalRead(encPinD)) ? -1 : 1;
}
void doEncoderD() {
  encoderPosRight += (digitalRead(encPinC) == digitalRead(encPinD)) ? 1 : -1;
}

class MotorControl{
public:
    void call(int x);
    int getInput();
};
 
void Calculation() {
    target_deg = (motorDegL + motorDegR) / 2  * (2 * M_PI * 11.5) / 360;

    // DC모터 왼쪽
    motorDegL = abs(encoderPosLeft * proportion);

    errorL = target_deg - motorDegL;

    delta_vL = kp_L * (errorL - error_prev_L) + ki_L * errorL + kd_L * (errorL - 2 * error_prev_L + error_prev_prev_L);
    controlL += delta_vL;

    error_prev_prev_L = error_prev_L;
    error_prev_L = errorL;

    // DC모터 오른쪽
    motorDegR = abs(encoderPosRight * proportion);

    errorR = target_deg - motorDegR;

    delta_vR = kp_R * (errorR - error_prev_R) + ki_R * errorR + kd_R * (errorR - 2 * error_prev_R + error_prev_prev_R);
    controlR += delta_vR;

    error_prev_prev_R = error_prev_R;
    error_prev_R = errorR;

    // 로봇의 선형 변위와 각변위 계산식 
    delta_s = 11.5 / 2 * (motorDegL + motorDegR);

    delta_deg = 11.5  / 29.2 * (motorDegL + motorDegR);

    // 로봇의 위치, 방향각을 좌표로 계산식 
    setha = setha_prev + delta_deg / 2;
    setha_prev = setha;

    x_coordinate = x_prev_coordinate + cos(setha) * delta_s;
    x_prev_coordinate = x_coordinate;

    y_coordinate = y_prev_coordinate + sin(setha) * delta_s;
    y_prev_coordinate = y_coordinate;

    deg_coordinate = deg_prev_coordinate + delta_deg;
    deg_prev_coordinate = deg_coordinate;

    // PID를 사용하기 위해서는 원하는 각도 혹은 거리를 비교하여 사용 -> 원하는 각도 혹은 거리 알아야 함 
    cout << "--------------------------------------------------------------------------------" << endl;
    cout << "왼쪽 모터의 회전각도 = " << motorDegL << "오른쪽 모터의 회전각도 = " << motorDegR << endl;
    cout << "로봇의 선형 변위 = " << delta_s<< endl;
    cout << "로봇의 각변위 =" << delta_deg << endl;
    cout << "로봇의 x좌표 =" << x_coordinate << "로봇의 y좌표 =" << y_coordinate << endl;
    cout << "로봇의 방향각 =" << deg_coordinate << endl;
}

// 원하는 방향 입력
int MotorControl::getInput() {
    int x;
    cout << "정지 : 0 / 직진 : 1 / 후진 : 2 / 오른쪽 : 3 / 왼쪽 : 4" << endl;
    cout << "원하는 방향을 입력하시오 : ";
    cin >> x;
    cout << "가고 싶은 거리를 입력하시오 : ";
    cin >> target_deg;
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
        // 속도 설정 
        softPwmWrite(pwmPinA, 0);
        softPwmWrite(pwmPinB, 0);    
        
        Calculation();

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
        digitalWrite(BIN3, HIGH);
        digitalWrite(BIN4, LOW);
        // 속도 설정 
        softPwmWrite(pwmPinA, 70);        // 만약에 동작 안 할 경우 255. -> 100. 으로 수정    
        softPwmWrite(pwmPinB, 70);          

        Calculation();       

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
        digitalWrite(BIN3, LOW);
        digitalWrite(BIN4, HIGH);
        // 속도 설정 
        softPwmWrite(pwmPinA, 70);    
        softPwmWrite(pwmPinB, 70);  

        Calculation();

        // x(방향)의 값이 2(후진)이 아닐 경우 x(방향)을 다시 입력 받음 
        if(x != 2){
            x = getInput();
        }   
    }

    // 오른쪽
    else if (x == 3){
        // 방향 조절 
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN3, HIGH);
        digitalWrite(BIN4, LOW);
        // 속도 설정 
        softPwmWrite(pwmPinA, 70);
        softPwmWrite(pwmPinB, 10);  

        Calculation();

        // x(방향)의 값이 3(오른쪽)이 아닐 경우 x(방향)을 다시 입력 받음 
        if(x != 3){
            x = getInput();
        }         
    }
           
    // 왼쪽
    else if (x == 4){
        // 방향 조절 
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN3, HIGH);
        digitalWrite(BIN4, LOW);
        // 속도 설정 
        softPwmWrite(pwmPinA, 10);
        softPwmWrite(pwmPinB, 70);  

        Calculation();

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

    while(true) {
        int lidar_way = control.getInput();
        control.call(lidar_way);
        delay(1000);
    }
    return 0;
}