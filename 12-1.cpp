/* 라이다 센서 연동 */
/* 두 모터의 속도 차이를 이용한 전/후/좌/우 회전 */ 
// 회전각도 구하기, 이동거리 구하기 -> 제어 확인
// 자이로 센서 이용하여 좌우각도 회전하기 

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


// 모터의 회전각도 구할 때 필요 
double target_turn_deg;
double de_A;
double de_B;
double di_A = 0;
double di_B = 0;
double dt = 0;



void Calculation() {

}

// 원하는 방향 입력
int MotorControl::getInput() {
    
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

        // 이동거리 출력 
        cout << "차체 중앙 기준 이동거리 = " << motor_distance_A << endl;
        cout << "encR = " << encoderPosRight << endl;
        cout << "deg = " << motorDegA << endl;
        // x(방향)의 값이 0(정지)이 아닐 경우 x(방향)을 다시 입력 받음 
        if(x != 0){
            x = getInput();
        }
    }
    
    // 전진
    else if (x == 1) {
        // 방향 설정 
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN3, HIGH);
        digitalWrite(BIN4, LOW); 
        delay(10);
        // 속도 설정 
        softPwmWrite(pwmPinA, min(abs(controlB), 100.));    
        softPwmWrite(pwmPinB, min(abs(controlB), 100.));  
        
        
        // x(방향)의 값이 1(전진)이 아닐 경우 x(방향)을 다시 입력 받음 
        if(x != 1){
            x = getInput();
        }
        else {
            // Calculation();
            delay(1000);
            // 이동거리 출력 
            //cout << "왼쪽 모터 이동거리  = " << motor_distance_A << endl;
            //cout << "왼쪽 모터 오차값 = " << derrorA << endl;
            cout << "오른쪽 모터 이동거리  = " << motor_distance_B << endl;
            cout << "오른쪽 모터 오차값 = " << derrorB << endl;
            cout << "모터 각도 "<< motorDegB << endl;

            cout << "ctrlA = " << controlA << ", degA = " << motorDegA << ", errA = " << errorA << ", disA = " << motor_distance_A << ", derrA = " << derrorA << endl;
            cout << "ctrlB = " << controlB << ", degB = " << motorDegB << ", errB = " << errorB << ", disB = " << motor_distance_B << ", derrB = " << derrorB << endl;
            cout << "encA = " << encoderPosLeft<< endl;
            cout << "encB = " << encoderPosRight << endl;

        }
    }
}
