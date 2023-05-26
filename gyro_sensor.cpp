#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>

#define MPU6050_ADDRESS 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B   // 가속도 데이터 읽기
#define GYRO_XOUT_H 0x43    // 각속도 데이터 읽기

int main() {
    wiringPiSetup();

    // I2C 통신을 위한 파일 디스크립터 가져오기
    int fd = wiringPiI2CSetup(MPU6050_ADDRESS);

    // MPU6050 초기화 및 동작 활성화
    wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0);

    while (1) {
        // 가속도 데이터 읽기
        int16_t accel_x = wiringPiI2CReadReg16(fd, ACCEL_XOUT_H);
        int16_t accel_y = wiringPiI2CReadReg16(fd, ACCEL_XOUT_H + 2);
        int16_t accel_z = wiringPiI2CReadReg16(fd, ACCEL_XOUT_H + 4);

        // 각속도 데이터 읽기
        int16_t gyro_x = wiringPiI2CReadReg16(fd, GYRO_XOUT_H);
        int16_t gyro_y = wiringPiI2CReadReg16(fd, GYRO_XOUT_H + 2);
        int16_t gyro_z = wiringPiI2CReadReg16(fd, GYRO_XOUT_H + 4);

        // 가속도 값 출력
        std::cout << "Accelerometer: X=" << accel_x << ", Y=" << accel_y << ", Z=" << accel_z << std::endl;

        // 각속도 값 출력
        std::cout << "Gyroscope: X=" << gyro_x << ", Y=" << gyro_y << ", Z=" << gyro_z << std::endl;

        delay(100);  // 100ms 지연
    }

    return 0;
}