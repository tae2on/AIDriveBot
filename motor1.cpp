/* 좌표를 이용한 PID값 */
/* 지금까지 찾은 오류 */
// 1. 처음에 값을 입력해도 정지 -> 해결
// 2. 원하는 거리까지 가고난 후 정지 안 함 -> 해결 
//   2-1. 거리가 더 감 -> 해결  
//   2-2. 전진*후진 PID 값 수정(거리) -> 전류 높이기
// 3. 초기 입력값 동작 확인 후 다음 입력값 동작 확인 -> 해결
// 4. 각도 동작할 때 멈추는 코드 확인 
// 5. 거리 PID, 각도 PID 값 맞추기 

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
#include <cmath>

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

/* PID 상수*/
// 각도 PID
float kp_dL = 5; // 8     //4.7
float ki_dL = 0; // 0.1   //0.4
float kd_dL = 10; // 0     //0.1 

float kp_dR = 5; // 거리 : 5
float ki_dR = 0; // 거리 : 0.4
float kd_dR = 10; // 0 

float kp_sL = 5; 
float kd_sL = 0;        
float ki_sL = 0; 
 
float kp_sR = 5;  
float kd_sR = 0;        
float ki_sR = 0; 

double difference = 1;
double tolerance = 0.2;
double sss = 0.0001;

volatile int encoderPosLeft = 0;              // 엔코더 값 - 왼쪽
volatile int encoderPosRight = 0;              // 엔코더 값 - 왼쪽 
 
int encoderPosLeft_prev = 0;
int encoderPosRight_prev = 0;

double motorDegL = 0;
double motorDegR = 0;

double motor_sethaL = 0;
double motor_sethaR = 0;

double rad = M_PI / 180;
double deg = 180 / M_PI;

/* 원하는 x,y 좌표값, 각도값 */
double x_target_coordinate;     // 수평
double y_target_coordinate;     // 수직
double setha_target;

/* 로봇의 선형 변위와 각변위 계산식 */
double delta_s = 0;
double delta_setha = 0;
double combine_delta_setha = 0;
double calculate_setha_target = 0;

/* 로봇의 위치, 방향각을 좌표로 계산식 */
double bar_setha = 0 ;
double bar_setha1 = 0 ; 
double round_bar_setha;

double x_coordinate = 0;
double x_prev_coordinate = 0;
double combine_x_coordinate = 0;

double y_coordinate = 0;
double y_prev_coordinate = 0;
double combine_y_coordinate = 0;

double setha_coordinate = 0;
double setha_prev_coordinate = 0;

/* 거리값, 각도값 계산식 */
double distance_robot; 
double distance_target;

double error_d = 0;
double error_prev_d = 0;
double error_prev_prev_d = 0;

double error_s = 0;
double error_prev_s = 0;
double error_prev_prev_s = 0;

double control_L = 0;
double delta_vL = 0;
double control_R = 0;
double delta_vR = 0;

/* 삭제 */
double del_ts = 0.01;
double e_setha_dot = 0;
double e_setha_total = 0;
double e_distance_dot = 0;
double e_distance_total = 0;

double delta_distanceL = 0;
double delta_distanceR = 0;
double prev_distance_robot = 0;

int pwmR;
int pwmL;

// 원하는 값 입력
struct InputData {
  double x_target_coordinate;
  double y_target_coordinate;
  double setha_target;
  double distance_target;
};

class MotorControl {
public:
  void call(InputData input);
  InputData getInput();
};

auto start = std::chrono::high_resolution_clock::now();  // 루프 시작 시간 기록

// 인터럽트 
void doEncoderA() {
  encoderPosLeft  += (digitalRead(encPinA) == digitalRead(encPinB)) ? 1 : -1;
}
void doEncoderB() {
  encoderPosLeft  += (digitalRead(encPinA) == digitalRead(encPinB)) ? -1 : 1;
}
void doEncoderC() {
  encoderPosRight += (digitalRead(encPinC) == digitalRead(encPinD)) ? 1 : -1;
}
void doEncoderD() {
  encoderPosRight += (digitalRead(encPinC) == digitalRead(encPinD)) ? -1 : 1;
}
 
void Calculation(InputData input) {
  motor_sethaL = encoderPosLeft * proportion;
  motor_sethaR = encoderPosRight * proportion;

  motorDegL = (encoderPosLeft - encoderPosLeft_prev) * proportion * rad;
  motorDegR = (encoderPosRight - encoderPosRight_prev) * proportion * rad;

  /* 로봇의 선형 변위와 각변위 계산식 */
  delta_s = (11.5 / 2) * (motorDegL + motorDegR);
  delta_setha = (11.5 / 74) * (motorDegR - motorDegL);
  combine_delta_setha += delta_setha;

  /* 로봇의 위치와 방향각 계산식 */
  bar_setha = ((combine_delta_setha - delta_setha) + (delta_setha / 2));
  bar_setha1 = (combine_delta_setha + (delta_setha / 2));

  round_bar_setha = std::round(bar_setha * 10) / 10;

  // DC모터 x좌표, y좌표
  x_coordinate = cos(round_bar_setha) * delta_s;
  y_coordinate = sin(round_bar_setha) * delta_s;
        
  combine_x_coordinate += x_coordinate; 
  combine_y_coordinate += y_coordinate;

  // DC모터 방향각//
  //setha_coordinate = setha_prev_coordinate + delta_setha;

  /* 거리값, 각도값 PID 계산식*/
  distance_target = sqrt(pow(input.x_target_coordinate, 2)+ pow(input.y_target_coordinate, 2)) + prev_distance_robot;
  distance_robot = sqrt(pow(combine_x_coordinate, 2) + pow(combine_y_coordinate, 2));

  encoderPosLeft_prev = encoderPosLeft;
  encoderPosRight_prev = encoderPosRight;

  // 입력으로 받은 각도를 사용하여 모터 회전 계산 -> SETHA_TARGET이 0이 아닐 경우로 코드 수정 
  if ((input.x_target_coordinate != 0) && (input.y_target_coordinate != 0) && (input.setha_target != 0) && (input.distance_target >= 0)) {
    if ((input.x_target_coordinate < 0) && (input.y_target_coordinate < 0)) {
      calculate_setha_target = -M_PI + atan2(input.x_target_coordinate, input.y_target_coordinate);
    }
    else if ((input.x_target_coordinate > 0) && (input.y_target_coordinate < 0)) {
      calculate_setha_target = M_PI + atan2(input.x_target_coordinate, input.y_target_coordinate);
    }
    else {
      calculate_setha_target = atan2(input.x_target_coordinate, input.y_target_coordinate);
    }
  }

  // 3사분면과 4사분면일 경우, 360도에서 각도를 뺀 값을 사용하여 회전
  if (calculate_setha_target > M_PI_2) {
    calculate_setha_target = 2 * M_PI - calculate_setha_target;
  }

  if (input.setha_target > 180.0) {
    calculate_setha_target = calculate_setha_target - ((2.0 * M_PI) - (input.setha_target * M_PI / 180.0));
    error_s = calculate_setha_target - combine_delta_setha;
  }
  else {
    error_s = input.setha_target - combine_delta_setha;
  }

  error_d = distance_target - distance_robot;
  e_setha_dot = (error_s - error_prev_s) / del_ts;
  e_setha_total = e_setha_total + error_s;
  e_distance_dot = (error_d - error_prev_d) / del_ts;
  e_distance_total = e_distance_total + error_d;

  cout << "--------------------------------------------------------------------------------" << endl;
  cout << "입력 x = " << input.x_target_coordinate << ", 입력 y = " << input.y_target_coordinate << endl;
  cout << "입력 setha = " << input.setha_target << ", 입력 거리 = " << input.distance_target << endl;
  cout << "거리 = " << distance_robot << endl;
  cout << "방향각 = "  << combine_delta_setha << endl;
  cout << "로봇 x = " << combine_x_coordinate << ", 로봇 y = " << combine_y_coordinate <<endl;
  cout << "degL = " << motor_sethaL << ", degR = " << motor_sethaR <<endl;
  cout << "encR = " << encoderPosRight << ", encL = " << encoderPosLeft << endl;
  cout << "ctrlL = " << control_L << ", ctrlR = " << control_R << endl;
  cout << "pwmL = " << pwmL << ", pwmR = " << pwmR << endl;
  cout << "과거 로봇 거리 = " << prev_distance_robot << ", distance_target = " << distance_robot << endl;
  cout << "error_d = " << error_d << ", error_prev_d = " << error_prev_d << ", error_prev_prev_d = " << error_prev_prev_d << endl;        
  cout << "error_s = " << error_s << endl;     

  // 왼쪽 DC모터 
  delta_distanceL = kp_dL * error_d + kd_dL * e_distance_dot;
  delta_vL = delta_distanceL + (kp_sL * error_s) + (ki_sL * e_setha_total) + (kd_sL * e_setha_dot);
  control_L = delta_vL;
  pwmL = abs(control_L);

  // 오른쪽 DC모터 
  delta_distanceR = kp_dR * error_d + kd_dR * e_distance_dot;
  delta_vR = delta_distanceR + (kp_sR * error_s) + (ki_sR * e_setha_total) + (kd_sR * e_setha_dot);
  control_R = delta_vR;
  pwmR = abs(control_R);

  // 이전값
  setha_prev_coordinate = setha_coordinate;
        
  error_prev_prev_d = error_prev_d;
  error_prev_d = error_d;

  error_prev_prev_s = error_prev_s;
  error_prev_s = error_s;

}

InputData MotorControl::getInput() {
  InputData input;

  std::cout << "x, y, setha, distance 값을 입력하시오: ";
  std::cin >> input.x_target_coordinate >> input.y_target_coordinate >> input.setha_target >> input.distance_target;

  return input;
}

void MotorControl::call(InputData input){
    while (true){
        // 전진
        if ((input.x_target_coordinate > 0) && (input.y_target_coordinate == 0) && (input.setha_target == 0) && (input.distance_target > 0)) {
            // 방향 설정 
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, HIGH);
            digitalWrite(BIN3, HIGH);
            digitalWrite(BIN4, LOW);
            
            // 속도 설정 
            softPwmWrite(pwmPinA, min(pwmL, 58));     
            softPwmWrite(pwmPinB, min(pwmR, 55));         

            Calculation(input);       
            
            if (error_d <= tolerance) {
                // 방향 설정 
                digitalWrite(AIN1, LOW);
                digitalWrite(AIN2, LOW);
                digitalWrite(BIN3, LOW);
                digitalWrite(BIN4, LOW);
                // 속도 설정 
                softPwmWrite(pwmPinA, 0);
                softPwmWrite(pwmPinB, 0);

                prev_distance_robot = distance_robot;

                auto end = std::chrono::high_resolution_clock::now();  // 루프 종료 시간 기록
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                std::cout << "지난 시간: " << duration.count() << "밀리초" << std::endl;
                del_ts = duration.count();

                input = getInput();    
            }
        }

        // 후진
        else if ((input.x_target_coordinate < 0) && (input.y_target_coordinate == 0) && (input.setha_target == 0) && (input.distance_target > 0)) {
            // 방향 설정 
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN3, LOW);
            digitalWrite(BIN4, HIGH);
            
            // 속도 설정 
            softPwmWrite(pwmPinA, min(pwmL, 58));     
            softPwmWrite(pwmPinB, min(pwmR, 55));         

            Calculation(input);       
            
            if (error_d <= tolerance) {
                // 방향 설정 
                digitalWrite(AIN1, LOW);
                digitalWrite(AIN2, LOW);
                digitalWrite(BIN3, LOW);
                digitalWrite(BIN4, LOW);
                // 속도 설정 
                softPwmWrite(pwmPinA, 0);
                softPwmWrite(pwmPinB, 0);

                prev_distance_robot = distance_robot;

                auto end = std::chrono::high_resolution_clock::now();  // 루프 종료 시간 기록
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                std::cout << "지난 시간: " << duration.count() << "밀리초" << std::endl;
                del_ts = duration.count();

                input = getInput();    
            }
        }

        // 제자리 회전 (왼쪽 방향으로 회전)
        else if ((input.x_target_coordinate == 0) && (input.y_target_coordinate == 0) && (input.setha_target > 0) && (input.distance_target == 0)) {
            // 방향 설정 
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN3, HIGH);
            digitalWrite(BIN4, LOW);
            
            // 속도 설정 
            softPwmWrite(pwmPinA, min(pwmL, 52));     
            softPwmWrite(pwmPinB, min(pwmR, 59));         

            Calculation(input);       
            
            if (error_s <= sss) {
                // 방향 설정 
                digitalWrite(AIN1, LOW);
                digitalWrite(AIN2, LOW);
                digitalWrite(BIN3, LOW);
                digitalWrite(BIN4, LOW);
                // 속도 설정 
                softPwmWrite(pwmPinA, 0);
                softPwmWrite(pwmPinB, 0);

                prev_distance_robot = distance_robot;

                auto end = std::chrono::high_resolution_clock::now();  // 루프 종료 시간 기록
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                std::cout << "지난 시간: " << duration.count() << "밀리초" << std::endl;
                del_ts = duration.count();

                input = getInput();    
            } 
        }
        // 제자리 회전 (오른쪽 방향으로 회전)
        else if ((input.x_target_coordinate == 0) && (input.y_target_coordinate == 0) && (input.setha_target < 0) && (input.distance_target == 0)) {
            // 방향 설정 
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, HIGH);
            digitalWrite(BIN3, LOW);
            digitalWrite(BIN4, HIGH);
            
            // 속도 설정 
            softPwmWrite(pwmPinA, min(pwmL, 52));     
            softPwmWrite(pwmPinB, min(pwmR, 59));         

            Calculation(input);       
            
            if (error_s <= sss) {
                // 방향 설정 
                digitalWrite(AIN1, LOW);
                digitalWrite(AIN2, LOW);
                digitalWrite(BIN3, LOW);
                digitalWrite(BIN4, LOW);
                // 속도 설정 
                softPwmWrite(pwmPinA, 0);
                softPwmWrite(pwmPinB, 0);

                prev_distance_robot = distance_robot;

                auto end = std::chrono::high_resolution_clock::now();  // 루프 종료 시간 기록
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                std::cout << "지난 시간: " << duration.count() << "밀리초" << std::endl;
                del_ts = duration.count();

                input = getInput();    
            }
        }
        // 회전
        else if ((input.x_target_coordinate != 0) && (input.y_target_coordinate != 0) && (input.setha_target != 0) && (input.distance_target >= 0)){
          // 1사분면 
          if ((input.x_target_coordinate > 0) && (input.y_target_coordinate > 0)){
            // 방향 조절 
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, HIGH);
            digitalWrite(BIN3, HIGH);
            digitalWrite(BIN4, LOW);
            
            // 속도 설정 
            softPwmWrite(pwmPinA, min(pwmL, 70));     
            softPwmWrite(pwmPinB, min(pwmR, 10));         

            Calculation(input);      
            
            std::cout << "회전 ";     
            
            if (error_s <= sss) {
                // 방향 설정 
                digitalWrite(AIN1, LOW);
                digitalWrite(AIN2, LOW);
                digitalWrite(BIN3, LOW);
                digitalWrite(BIN4, LOW);
                // 속도 설정 
                softPwmWrite(pwmPinA, 0);
                softPwmWrite(pwmPinB, 0);

                if (error_d > tolerance) {
                  // 방향 설정 
                  digitalWrite(AIN1, LOW);
                  digitalWrite(AIN2, HIGH);
                  digitalWrite(BIN3, LOW);
                  digitalWrite(BIN4, HIGH);
            
                  // 속도 설정 
                  softPwmWrite(pwmPinA, min(pwmL, 52));     
                  softPwmWrite(pwmPinB, min(pwmR, 59));      
                  
                  std::cout << "회전 후 전진 ";          
                }    
                  else if (error_d <= tolerance) {
                    // 방향 설정 
                    digitalWrite(AIN1, LOW);
                    digitalWrite(AIN2, LOW);
                    digitalWrite(BIN3, LOW);
                    digitalWrite(BIN4, LOW);
                    // 속도 설정 
                    softPwmWrite(pwmPinA, 0);
                    softPwmWrite(pwmPinB, 0);  

                    prev_distance_robot = distance_robot;

                    std::cout << "회전 후 정지 ";                  

                    auto end = std::chrono::high_resolution_clock::now();  // 루프 종료 시간 기록
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                    std::cout << "지난 시간: " << duration.count() << "밀리초" << std::endl;
                    del_ts = duration.count();

                    input = getInput();
                  }
            }        
        }

        // 2사분면 
          if ((input.x_target_coordinate < 0) && (input.y_target_coordinate > 0)){
            // 방향 조절 
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, HIGH);
            digitalWrite(BIN3, HIGH);
            digitalWrite(BIN4, LOW);
            
            // 속도 설정 
            softPwmWrite(pwmPinA, min(pwmL, 10));     
            softPwmWrite(pwmPinB, min(pwmR, 70));         

            Calculation(input);       
            
            if (error_s <= sss) {
                // 방향 설정 
                digitalWrite(AIN1, LOW);
                digitalWrite(AIN2, LOW);
                digitalWrite(BIN3, LOW);
                digitalWrite(BIN4, LOW);
                // 속도 설정 
                softPwmWrite(pwmPinA, 0);
                softPwmWrite(pwmPinB, 0);

                if (error_d > tolerance) {
                  // 방향 설정 
                  digitalWrite(AIN1, LOW);
                  digitalWrite(AIN2, HIGH);
                  digitalWrite(BIN3, LOW);
                  digitalWrite(BIN4, HIGH);
            
                  // 속도 설정 
                  softPwmWrite(pwmPinA, min(pwmL, 52));     
                  softPwmWrite(pwmPinB, min(pwmR, 59));      
                  
                  std::cout << "회전 후 전진 ";          
                }    
                  else if (error_d <= tolerance) {
                    // 방향 설정 
                    digitalWrite(AIN1, LOW);
                    digitalWrite(AIN2, LOW);
                    digitalWrite(BIN3, LOW);
                    digitalWrite(BIN4, LOW);
                    // 속도 설정 
                    softPwmWrite(pwmPinA, 0);
                    softPwmWrite(pwmPinB, 0);  

                    prev_distance_robot = distance_robot;

                    std::cout << "회전 후 정지 ";                  

                    auto end = std::chrono::high_resolution_clock::now();  // 루프 종료 시간 기록
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                    std::cout << "지난 시간: " << duration.count() << "밀리초" << std::endl;
                    del_ts = duration.count();

                    input = getInput();
                  }
            }               
        }
        // 3사분면 
          if ((input.x_target_coordinate < 0) && (input.y_target_coordinate < 0)){
            // 방향 조절 
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN3, LOW);   
            digitalWrite(BIN4, HIGH);
            
            // 속도 설정 
            softPwmWrite(pwmPinA, min(pwmL, 10));     
            softPwmWrite(pwmPinB, min(pwmR, 70));         

            Calculation(input);       
            
            if (error_s <= sss) {
                // 방향 설정 
                digitalWrite(AIN1, LOW);
                digitalWrite(AIN2, LOW);
                digitalWrite(BIN3, LOW);
                digitalWrite(BIN4, LOW);
                // 속도 설정 
                softPwmWrite(pwmPinA, 0);
                softPwmWrite(pwmPinB, 0);

                if (error_d > tolerance) {
                  // 방향 설정 
                  digitalWrite(AIN1, LOW);
                  digitalWrite(AIN2, HIGH);
                  digitalWrite(BIN3, LOW);
                  digitalWrite(BIN4, HIGH);
            
                  // 속도 설정 
                  softPwmWrite(pwmPinA, min(pwmL, 52));     
                  softPwmWrite(pwmPinB, min(pwmR, 59));      
                  
                  std::cout << "회전 후 전진 ";          
                }    
                  else if (error_d <= tolerance) {
                    // 방향 설정 
                    digitalWrite(AIN1, LOW);
                    digitalWrite(AIN2, LOW);
                    digitalWrite(BIN3, LOW);
                    digitalWrite(BIN4, LOW);
                    // 속도 설정 
                    softPwmWrite(pwmPinA, 0);
                    softPwmWrite(pwmPinB, 0);  

                    prev_distance_robot = distance_robot;

                    std::cout << "회전 후 정지 ";                  

                    auto end = std::chrono::high_resolution_clock::now();  // 루프 종료 시간 기록
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                    std::cout << "지난 시간: " << duration.count() << "밀리초" << std::endl;
                    del_ts = duration.count();

                    input = getInput();
                  }
            }  
        }
        // 4사분면 
          if ((input.x_target_coordinate > 0) && (input.y_target_coordinate < 0)){
            // 방향 조절 
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN3, LOW);   
            digitalWrite(BIN4, HIGH);
            
            // 속도 설정 
            softPwmWrite(pwmPinA, min(pwmL, 10));     
            softPwmWrite(pwmPinB, min(pwmR, 70));         

            Calculation(input);       
            
            if (error_s <= sss) {
                // 방향 설정 
                digitalWrite(AIN1, LOW);
                digitalWrite(AIN2, LOW);
                digitalWrite(BIN3, LOW);
                digitalWrite(BIN4, LOW);
                // 속도 설정 
                softPwmWrite(pwmPinA, 0);
                softPwmWrite(pwmPinB, 0);

                prev_distance_robot = distance_robot;

                auto end = std::chrono::high_resolution_clock::now();  // 루프 종료 시간 기록
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                std::cout << "지난 시간: " << duration.count() << "밀리초" << std::endl;
                del_ts = duration.count();

                input = getInput();    
            }
        }
    }
  }
}

int main(){
    wiringPiSetup();

    MotorControl control;
    InputData input;

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

    input = control.getInput();  // 초기 입력값 받기
    control.call(input);         // 초기 입력값으로 모터 동작 및 계산 수행

    std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();  // 루프 시작 시간 기록

    while(true) {
      input = control.getInput();
      control.call(input);
      delay(1000);
    }
  return 0;
}    