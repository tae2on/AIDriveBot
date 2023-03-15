/* 라이다 센서 연동 */ 
// 1. control -> 실수 ?로 바꿔도 되나 ? 
// 2. 함수도 선언해주어야 하나 ? 

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <wiringPi.h>               // analogRead(), pinMode(), delay() 함수 등 사용 
#include <iostream>                 // C++ 입출력 라이브러리
#include <ctime>                    // 현재 시간 사용 
#include <cmath>                    // pi 사용 
#include <thread>
#include <chrono>
// #include <pigpio.h>

using namespace std;

#define M_PI 3.14159265358979323846

/* 핀 번호가 아니라 GPIO 번호 ! ----------------------------> 핀 설정 다시하기 
   DC 모터 왼쪽 (엔코더 O) */                                                     
#define pwmPinA 15      // 모터드라이버 ENA / ex) 핀 번호 8번, GPIO 14번
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

/* PID 상수 */
float kp = 30.0; 
float kd = 0.;         
float ki = 0.;

float encoderPosLeft = 0;
float encoderPosRight = 0;

float motorDegA = 0;
float motorDegB = 0;
float errorA = 0;
float errorB = 0;

float controlA = 0.;
float controlB = 0.;

int pwm;
int encoderPosA = 0;
int encoderPosB = 0;

double wheel;
double target_deg;                      // 목표 각도 
double target_direction = 0.;           // 목표 방향 
double target_distance = 0.;            // 목표 거리 
double dt = 0.;
double dt_sleep = 0.01;
double tolerance = 0.1;
double time_prev = 0.;

/* DC모터 왼쪽 */
double de_A;
double di_A = 0.;
double error_prev_A = 0.;
double error_prev_prev_A = 0.;
double delta_vA;
double motor_distanceA;
double derrorA;

/* DC모터 오른쪽 */
double de_B;
double di_B = 0.;
double error_prev_B = 0.;
double error_prev_prev_B = 0.;
double delta_vB;
double motor_distanceB;
double derrorB;

int frequency = 100;                // PWM 주파수

gpioHardwarePWM(pwmPinA, frequency, 0);        // PWM 시작
gpioHardwarePWM(pwmPinB, frequency, 0);

gpioPWM(pwmPinA, 0);                        // duty cycle을 0으로 설정
gpioPWM(pwmPinB, 0);

time_t start_time = time(nullptr);

/* 전진 */
void goFront() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN3, LOW);
    digitalWrite(BIN4, HIGH);
    delay(10);
    analogWrite(pwmPinA, min(abs(controlA), 255.));
    analogWrite(pwmPinB, min(abs(controlA), 255.));
    // gpioPWM(pwmPinB, min(abs(int(controlA)), 255));
    
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
    analogWrite(pwmPinA, min(abs(controlA), 255.));
    analogWrite(pwmPinB, min(abs(controlA), 255.));

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
    pwmWrite(p1, 0);
    pwmWrite(p2, 0);

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

/* ---------------------------- 모터의 이동거리 ---------------------------- */
/* 모터 이동거리 (r: 11.5) */
int main () {
    wiringPiSetup();
    
    gpioSetMode(pwmPinA, PI_OUTOUT);
    gpioSetMode(pwmPinB, PI_OUTOUT);

    pinMode(encPinA, INPUT_PULLUP);
    pinMode(encPinB, INPUT_PULLUP);
    pinMode(encPinC, INPUT_PULLUP);
    pinMode(encPinD, INPUT_PULLUP);
    pinMode(pwmPinA, OUTPUT);
    pinMode(pwmPinB, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN3, OUTPUT);
    pinMode(BIN4, OUTPUT);

    interruptInit();
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN3, LOW);
    digitalWrite(BIN4, LOW);

    analogWrite(pwmPinA, 0);  
    analogWrite(pwmPinB, 0); 

    /* 인터럽트 */  
    void encoderA(encPinA) {
        encoderPosLeft += (digitalRead(encPinA) == digitalRead(encPinB)) ? 1 : -1;
    }
    void encoderB(encPinB) {
        encoderPosLeft += (digitalRead(encPinA) == digitalRead(encPinB)) ? -1 : 1;
    }
    void encoderC(encPinC) {
        encoderPosRight += (digitalRead(encPinC) == digitalRead(encPinD))? 1 : -1;
    }
    void encoderD(encPinD) {
        encoderPosRight += (digitalRead(encPinC) == digitalRead(encPinD)) ? -1 : 1;
    }

    wiringPiISR(PhaseA, INT_EDGE_BOTH, &encoderA);
    wiringPiISR(PhaseB, INT_EDGE_BOTH, &encoderB);
    wiringPiISR(PhaseC, INT_EDGE_BOTH, &encoderC);
    wiringPiISR(PhaseD, INT_EDGE_BOTH, &encoderD);

    while (true) {
        wheel = 2*M_PI*11.5;
        target_deg = (360*target_distance / wheel) ;      // 목표 각도
        
        /* DC모터 왼쪽 */
        motorDegA = encoderPosA * ratio;                        // 모터 움직인 각도
        errorA = target_deg - motorDegA;                        // 각도 오차값 
        de_A = errorA - error_prev_A;
        di_A += errorA * dt;
        dt = std::time(nullptr) - time_prev;

        delta_vA = kp*de_A + ki*errorA + kd*(errorA - 2*error_prev_A + error_prev_prev_A);
        controlA += delta_vA;
        error_prev_A = errorA;
        error_prev_prev_A = error_prev_A;

        motor_distanceA = motorDegA * wheel / 360;           // 모터 움직인 거리
        derrorA = abs(target_distance - motor_distanceA);    // 거리 오차값

        /* DC 모터 오른쪽 */
        motorDegB = abs(encoderPosB * ratio);               // 모터 움직인 각도
        errorB = target_deg - motorDegB;                    // 각도 오차값 
        de_B = errorB - error_prev_B;
        di_B += errorB * dt;
        dt = std::time(nullptr) - time_prev;

        delta_vB = kp*de_B + ki*errorB + kd*(errorB - 2*error_prev_B + error_prev_prev_B);
        controlB += delta_vB;
        error_prev_B = errorB;
        error_prev_prev_B = error_prev_B;

        motor_distanceB = motorDegB * wheel / 360;          // 모터 움직인 거리
        derrorB = abs(target_distance - motor_distanceB);   // 거리 오차값
    }

    call("goFront");
    delay(1000);
    call("Stop");
    delay(1000);
    call("goBack");
    delay(1000);
    call("Stop");
    delay(1000);
}

p1.join()
p2.join() 