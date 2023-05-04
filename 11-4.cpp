#include <iostream>
#include <wiringPi.h>
#include <wiringPiEncoder.h>

// 엔코더 핀 설정
const int encoderPinA = 2; // 핀 A (BCM 번호 사용)
const int encoderPinB = 3; // 핀 B (BCM 번호 사용)

int main() {
  // wiringPi 초기화 및 핀 모드 설정
  wiringPiSetupGpio();
  int encoder = encoderSetup(encoderPinA, encoderPinB);

  if (encoder < 0) {
    std::cerr << "Encoder setup failed!" << std::endl;
    return 1;
  }

  long previousEncoderValue = 0; // 이전 엔코더 값 저장 변수

  std::cout << "Rotary Encoder Test:" << std::endl;

  while (true) {
    long currentEncoderValue = encoderGet(encoder); // 현재 엔코더 값 읽기

    // 이전 엔코더 값과 현재 값이 다를 경우 출력
    if (currentEncoderValue != previousEncoderValue) {
      std::cout << "Encoder value: " << currentEncoderValue << std::endl;
      previousEncoderValue = currentEncoderValue; // 이전 값 업데이트
    }
  }

  return 0;
}