#include "wiringPi.h"
#include <softPwm.h>
#include <iostream>

#define pwmPinA 25             // 모터드라이d버 ENA - GPIO핀 번호: 12
#define AIN1 22            // IN1 - GPIO핀 번호: 16
#define AIN2 21            // IN2 - GPIO핀 번호 : 25 
#define encPinA 3           // 보라색 (A) - GPIO핀 번호 : 23
#define encPinB 6           // 파랑색 (B) - GPIO핀 번호 : 24

volatile int pulse_count = 0;

void pulse_callback() {
    pulse_count++;
}

int main() {
    wiringPiSetup();

    pinMode(encPinA, INPUT);

    pinMode(pwmPinA, OUTPUT); 
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    softPwmCreate(pwmPinA, 0, 100);
    softPwmWrite(pwmPinA, 50);

    digitalWrite(pwmPinA, LOW);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    
    wiringPiISR(encPinA, INT_EDGE_BOTH, &pulse_callback);

    while (1) {
        pulse_callback();
        std::cout << "Pulse Count: " << pulse_count << std::endl;

        if (pulse_count == 20){
            softPwmWrite(pwmPinA, 0);
        }

    }

    return 0;
}