#include <iostream>
#include <wiringPi.h>
#include <wiringPiEncoder.h>

// 엔코더 핀 설정
const int encoderPinA = 2; // 핀 A (BCM 번호 사용)
const int encoderPinB = 3; // 핀 B (BCM 번호 사용)

// 모터 제어 핀 설정
const int motorPin1 = 4; // 모터 핀 1 (BCM 번호 사용)
const int motorPin2 = 5; // 모터 핀 2 (BCM 번호 사용)

void setupMotorPins() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
}

void rotateMotor(int direction) {
  if (direction > 0) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }
}

void stopMotor() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}

int main() {
  // wiringPi 초기화 및 핀 모드 설정
  wiringPiSetupGpio();
  setupMotorPins();

  int encoder = encoderSetup(encoderPinA, encoderPinB);

  if (encoder < 0) {
    std::cerr << "Encoder setup failed!" << std::endl;
    return 1;
  }

  long previousEncoderValue = 0; // 이전 엔코더 값 저장 변수

  std::cout << "Rotary Encoder Test:" << std::endl;

  // 모터를 회전시킵니다.
  rotateMotor(1);

  for (int i = 0; i < 1000; ++i) {
    long currentEncoderValue = encoderGet(encoder); // 현재 엔코더 값 읽기

    // 이전 엔코더 값과 현재 값이 다를 경우 출력
    if (currentEncoderValue != previousEncoderValue) {
      std::cout << "Encoder value: " << currentEncoderValue << std::endl;
      previousEncoderValue = currentEncoderValue; // 이전 값 업데이트
    }

    delay(10); // 10ms 지연
  }

  // 모터를 정지시킵니다.
  stopMotor();

  return 0;
}