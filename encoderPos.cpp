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
#define encPinA 5           // 보라색 (A) - GPIO핀 번호 : 23
#define encPinB 4           // 파랑색 (B) - GPIO핀 번호 : 24

int encoderPosLeft = 0;              // 엔코더 값 - 왼쪽
int lidar_way;
int x;

// 인터럽트 
void doEncoderA() {
  encoderPosLeft  += (digitalRead(encPinA) == digitalRead(encPinB)) ? -1 : 1;
}
void doEncoderB() {
  encoderPosLeft  += (digitalRead(encPinA) == digitalRead(encPinB)) ? 1 : -1;
}

class MotorControl{
public:
    void call(int x);
    int getInput();
};
 
// 원하는 방향 입력
int MotorControl::getInput() {
    int x;
    cout << "정지 : 0 / 직진 : 1" << endl;
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

        // 속도 설정 
        softPwmWrite(pwmPinA, 0);
  
        
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

        // 속도 설정 
        softPwmWrite(pwmPinA, 10);        // 만약에 동작 안 할 경우 255. -> 100. 으로 수정           

        cout << "--------------------------------------------------------------------------------" << endl;
        cout << "encA = " << encoderPosLeft << endl;    

        // x(방향)의 값이 1(전진)이 아닐 경우 x(방향)을 다시 입력 받음 
        if(x != 1){
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

    while (true){

        int lidar_way = control.getInput();
        control.call(lidar_way);
        delay(1000);   

        cout << "--------------------------------------------------------------------------------" << endl;
        cout << "encA = " << encoderPosLeft << endl;
            
        if (encoderPosLeft >= 3360){
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, LOW);

            softPwmWrite(pwmPinA, 0); 

            delay(1000); 
        }
    }    
    return 0; 
}
