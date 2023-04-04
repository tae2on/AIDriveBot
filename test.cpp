/*
<MOTER POSITION>            <MOTER DRIVE POSITION>
FL-----FR                   ENB IN4 IN3 IN2 IN1 ENA | ENB IN4 IN3 IN2 IN1 ENA
|       |                     3   2   0   6   5   4 |  29  28  27  21  22  23
|       |                       <BR>    |   <FR>    |     <BL>        <FL>      
|       |
BL-----BR
*/

#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>
#include <unistd.h>

using namespace std;

// FL-MOTER
#define EN_FL 23    
#define IN_FL_1 22
#define IN_FL_2 21
int OUT_FL_1 = LOW;
int OUT_FL_2 = LOW;

// BL-MOTER
#define EN_BL 29
#define IN_BL_1 28
#define IN_BL_2 27
int OUT_BL_1 = LOW;
int OUT_BL_2 = LOW;


 void state_value(string state) {
    if (state == "ready") {
      OUT_FL_1 = OUT_FL_2 = OUT_BL_1 =  OUT_BL_2 = LOW;
      softPwmWrite(EN_FL, 0);
      softPwmWrite(EN_BL, 0);
    }
    else if (state == "go") {
      OUT_FL_1 = OUT_BL_1 = LOW;
      OUT_FL_2 = OUT_BL_2 = HIGH;
      softPwmWrite(EN_FL, 30);
      softPwmWrite(EN_BL, 100);

    }
    else if (state == "back") {
      OUT_FL_1 = OUT_BL_1 =  HIGH;
      OUT_FL_2 = OUT_BL_2 = LOW;
      softPwmWrite(EN_FL, 100);
      softPwmWrite(EN_BL, 100);

    }
    else if (state == "right") {
      OUT_FL_1 = OUT_BL_1 = HIGH;
      OUT_FL_2 = OUT_BL_2 = LOW;
      softPwmWrite(EN_FL, 100);
      softPwmWrite(EN_BL, 100);

    }
    else if (state == "left") {
      OUT_FL_2 = OUT_BL_2 = HIGH;
      OUT_FL_1 = OUT_BL_1 = LOW;
      softPwmWrite(EN_FL, 100);
      softPwmWrite(EN_BL, 100);

    }
  }

int main() {

  // wiringPi 초기화
  wiringPiSetup();
  
  // FL-MOTER
  pinMode(EN_FL, OUTPUT);
  pinMode(IN_FL_1, OUTPUT);
  pinMode(IN_FL_2, OUTPUT);
  softPwmCreate(EN_FL, 0, 100);
  softPwmWrite(EN_FL, 0);

  // BL-MOTER
  pinMode(EN_BL, OUTPUT);
  pinMode(IN_BL_1, OUTPUT);
  pinMode(IN_BL_2, OUTPUT);
  softPwmCreate(EN_BL, 0, 100);
  softPwmWrite(EN_BL, 0);


  string input;

  while (1) {
    cout << "ready / go / back / right / left" << endl;
    cin >> input;
    state_value(input);
    sleep(0.3);
    digitalWrite(IN_FL_1, OUT_FL_1);
    digitalWrite(IN_FL_2, OUT_FL_2);
    digitalWrite(IN_BL_1, OUT_BL_1);
    digitalWrite(IN_BL_2, OUT_BL_2);

    sleep(5);
    state_value("ready");
  }
  return 0;
}