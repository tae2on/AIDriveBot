#include "wiringPi.h"
#include <softPwm.h>
#include <iostream>

#define pwmPinA 25             // 모터드라이d버 ENA - GPIO핀 번호: 12
#define AIN1 22            // IN1 - GPIO핀 번호: 16
#define AIN2 21            // IN2 - GPIO핀 번호 : 25 
#define encPinA 3           // 보라색 (A) - GPIO핀 번호 : 23
#define encPinB 0           // 파랑색 (B) - GPIO핀 번호 : 24

volatile int pulse_countA = 0;
volatile int pulse_countB = 0;

void pulse_callbackA() {
    if(pulse_countA < 11){
        pulse_countA++;
    }
}

void pulse_callbackB() {
    pulse_countB++;
      
}

int main() {
    wiringPiSetup();

    pinMode(encPinA, INPUT);
    pinMode(encPinB, INPUT);

    pinMode(pwmPinA, OUTPUT); 
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    softPwmCreate(pwmPinA, 0, 100);
    softPwmWrite(pwmPinA, 20);

    digitalWrite(pwmPinA, LOW);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    
    wiringPiISR(encPinA, INT_EDGE_RISING, &pulse_callbackA);
    wiringPiISR(encPinB, INT_EDGE_RISING, &pulse_callbackB);

    while (1) {
        pulse_callbackA();
        std::cout << "Pulse CountA: " << pulse_countA << std::endl;

        pulse_callbackB();
        std::cout << "Pulse CountB: " << pulse_countB << std::endl;
        delay(100);

        if (pulse_countA == 10){

            softPwmWrite(pwmPinA, 0);   
            pulse_countA = 0;    
            std::cout << "Pulse CountA: " << pulse_countA << std::endl; 
        }

    }

    return 0;
}