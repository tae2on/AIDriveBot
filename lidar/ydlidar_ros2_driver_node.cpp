/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <unistd.h>

// motor include start
#include "wiringPi.h"
#include <softPwm.h>
#include <algorithm>
#include <thread>
#include <ctime>
// motor include end

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

#include <time.h>
#include <fstream>
#include <csignal>
#include <stdlib.h>

#define ROS2Verision "1.0.1"

// #define RAD2DEG(x) ((x)*180./M_PI)

using namespace std;

// motor value start
// DC 모터 왼쪽 (엔코더 O)
#define pwmPinA 23              // 모터드라이버 ENA - GPIO핀 번호: 13 
#define AIN1 22                 // IN1 - GPIO핀 번호: 6
#define AIN2 21                 // IN2 - GPIO핀 번호 : 5
#define encPinA 8               // 보라색 (A) - GPIO핀 번호 : 2
#define encPinB 9               // 파랑색 (B) - GPIO핀 번호 : 3

// DC모터 오른쪽 (엔코더 X) 
#define pwmPinB 29              // 모터 드라이버 ENB - GPIO핀 번호 : 21
#define BIN3 7                  // IN3 - GPIO핀 번호 : 4
#define BIN4 27                 // IN4 - GPIO핀 번호 : 16
#define encPinC 2               // 보라색 (C) - GPIO핀 번호 : 27
#define encPinD 3               // 파랑색 (D) - GPIO핀 번호 : 22

float encoderPosRight = 0;             // 엔코더 값 - 오른쪽
float encoderPosLeft = 0;              // 엔코더 값 - 왼쪽

float motorDegA = 0;                   // 모터 각도A
float motorDegB = 0;                   // 모터 각도B
float motor_distanceA;                 // 모터 거리 
float motor_distanceB;                 // 모터 거리 

float errorA = 0;
float errorB = 0;
float error_prev_A = 0.;
float error_prev_B = 0.;
float error_prev_prev_A = 0;
float error_prev_prev_B = 0;
float derrorA;
float derrorB;

double controlA = 0.;
double controlB = 0.;
 
double wheel; 
double target_deg;                      // 목표 각도 
double target_direction = 0.;           // 목표 방향 
double target_distance = 0.;            // 목표 거리 

double de_A;
double de_B;
double di_A = 0;
double di_B = 0;
double dt = 0;

double delta_vA;
double delta_vB;
double time_prev = 0;

std::time_t start_time = std::time(nullptr);

/* 인터럽트 */
void doEncoderA() {
    encoderPosLeft += (digitalRead(encPinA) == digitalRead(encPinB)) ? 1 : >
}
void doEncoderB() {
    encoderPosLeft += (digitalRead(encPinA) == digitalRead(encPinB)) ? -1 :>
}
void doEncoderC() {
    encoderPosRight += (digitalRead(encPinC) == digitalRead(encPinD)) ? 1 :>
}
void doEncoderD() {
    encoderPosRight += (digitalRead(encPinC) == digitalRead(encPinD)) ? -1 >
}

//motor value end

//motor class
class MotorControl {
public:
    void Call(int x);
};


void MotorControl::Call(int x){
    // 정지
    if (x == 0){
        // 방향 설정 
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN3, LOW);
        digitalWrite(BIN4, LOW);

        // 속도 설정 
        softPwmWrite(pwmPinA, 0);
        softPwmWrite(pwmPinB, 0);    
    }
    
    // 전진
    else if (x == 1) {
        // 방향 설정 
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN3, LOW);
        digitalWrite(BIN4, HIGH);

        // 속도 설정 
        softPwmWrite(pwmPinA, 255.);
        softPwmWrite(pwmPinB, 220.);   
    }

   // 후진
    else if (x == 2) {
        // 방향 설정 
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN3, HIGH);
        digitalWrite(BIN4, LOW);   

        // 속도 설정 
        softPwmWrite(pwmPinA, 100.);
        softPwmWrite(pwmPinB, 100.);   
    }

    // 오른쪽
    else if (x == 3){
        // 목표 각도 입력
        //cout << "원하는 각도를 입력하시오 : ";
        //cin >> target_turn_deg;

        // 방향 조절 
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN3, LOW);
        digitalWrite(BIN4, HIGH);

        // 속도 설정 
        softPwmWrite(pwmPinA, 255);
        softPwmWrite(pwmPinB, 25);        
    }
   // 왼쪽
    else if (x == 4){
        // 목표 각도 입력 
        //cout << "원하는 각도를 입력하시오 : ";
        //cin >> target_turn_deg;        

        // 방향 조절 
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN3, LOW);
        digitalWrite(BIN4, HIGH);

        // 속도 설정 
        softPwmWrite(pwmPinA, 25);
        softPwmWrite(pwmPinB, 255);     
    }
}


// motor class

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  wiringPiSetup();

  MotorControl motor;

   //motor code start
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
   
    softPwmCreate(pwmPinA, 0, 255);
    softPwmCreate(pwmPinB, 0, 255);
    softPwmWrite(pwmPinA, 0);
    softPwmWrite(pwmPinB, 0); 

    digitalWrite(pwmPinA, LOW);
    digitalWrite(pwmPinB, LOW);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN3, LOW);
    digitalWrite(BIN4, LOW);

    softPwmCreate(pwmPinA, 0, 255);
    softPwmCreate(pwmPinB, 0, 255);
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
    //motor code end

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s\n", ((std::string)ROS2Verision).c_str());

  CYdLidar laser;
  std::string str_optvalue = "/dev/ydlidar";
  node->declare_parameter("port");
  node->get_parameter("port", str_optvalue);
  /// lidar port
  laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

  /// ignore array
  str_optvalue = "";
  node->declare_parameter("ignore_array");
  node->get_parameter("ignore_array", str_optvalue);
  laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

  std::string frame_id = "laser_frame";
  node->declare_parameter("frame_id");
  node->get_parameter("frame_id", frame_id);

  //////////////////////int property/////////////////
  /// lidar baudrate
  int optval = 230400;
  node->declare_parameter("baudrate");
  node->get_parameter("baudrate", optval);
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
  /// tof lidar
  optval = TYPE_TRIANGLE;
  node->declare_parameter("lidar_type");
  node->get_parameter("lidar_type", optval);
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
  node->declare_parameter("device_type");
  node->get_parameter("device_type", optval);
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = 5;
  node->declare_parameter("sample_rate");
  node->get_parameter("sample_rate", optval);
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  node->declare_parameter("abnormal_check_count");
  node->get_parameter("abnormal_check_count", optval);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  node->declare_parameter("fixed_resolution");
  node->get_parameter("fixed_resolution", b_optvalue);
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  b_optvalue = true;
  node->declare_parameter("reversion");
  node->get_parameter("reversion", b_optvalue);
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  b_optvalue = true;
  node->declare_parameter("inverted");
  node->get_parameter("inverted", b_optvalue);
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  node->declare_parameter("auto_reconnect");
  node->get_parameter("auto_reconnect", b_optvalue);
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = false;
  node->declare_parameter("isSingleChannel");
  node->get_parameter("isSingleChannel", b_optvalue);
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  b_optvalue = false;
  node->declare_parameter("intensity");
  node->get_parameter("intensity", b_optvalue);
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = false;
  node->declare_parameter("support_motor_dtr");
  node->get_parameter("support_motor_dtr", b_optvalue);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  node->declare_parameter("angle_max");
  node->get_parameter("angle_max", f_optvalue);
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  node->declare_parameter("angle_min");
  node->get_parameter("angle_min", f_optvalue);
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  /// unit: m
  f_optvalue = 16.f;
  node->declare_parameter("range_max");
  node->get_parameter("range_max", f_optvalue);
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.1f;
  node->declare_parameter("range_min");
  node->get_parameter("range_min", f_optvalue);
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  f_optvalue = 5.f;
  node->declare_parameter("frequency");
  node->get_parameter("frequency", f_optvalue);
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

  bool invalid_range_is_inf = false;
  node->declare_parameter("invalid_range_is_inf");
  node->get_parameter("invalid_range_is_inf", invalid_range_is_inf);

  bool ret = laser.initialize();
  if (ret) {
    ret = laser.turnOn();
  }
  else {
    RCLCPP_ERROR(node->get_logger(), "%s\n", laser.DescribeError());
  }

  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  auto stop_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOff();
  };

  auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan", stop_scan_service);

  auto start_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOn();
  };

  auto start_service = node->create_service<std_srvs::srv::Empty>("start_scan", start_scan_service);

  rclcpp::WallRate loop_rate(20);

  while (ret && rclcpp::ok()) {

    LaserScan scan; //

    if (laser.doProcessSimple(scan)) {

      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

      scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
      scan_msg->header.stamp.nanosec = scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      scan_msg->header.frame_id = frame_id;
      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;

      float range_data[1050] = {0};

      int size = (scan.config.max_angle - scan.config.min_angle) / scan.config.angle_increment + 1;
      scan_msg->ranges.resize(size);
      scan_msg->intensities.resize(size);

      for (size_t i = 0; i < scan.points.size(); i++) {
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle) / scan.config.angle_increment);
        if (index >= 0 && index < size) {
          scan_msg->ranges[index] = scan.points[i].range;
          scan_msg->intensities[index] = scan.points[i].intensity;

          range_data[i] = scan.points[i].range;
        }
      }

      // my code start
      //data input code start
      const int num_arrays = 360;
      //float arr_avg[num_arrays] = {0};
      int points = scan.points.size();
      int arr_size = sizeof(range_data) / sizeof(range_data[0]);
      std::vector<float> data(range_data, range_data + arr_size);
      vector<vector<float>> arrays(num_arrays);
      //int elements_per_array = points / num_arrays;
      int index = 0;
      bool angle_cnt[8] = { false };
      bool cnt_a[8] = { false };
      float SOF = 0.0;
      float SOB = 0.0;
      int Fcnt = 0;
      int Bcnt = 0;
      int Flen = 0;
      int flen = 0;

for (int j = 0; j < num_arrays; j++) {
              int num_elements = points / num_arrays;
              if (j < points % num_arrays) {
                  num_elements++;
              }
              for (int k = 0; k < num_elements; k++) {
                  arrays[j].push_back(data[index]);
                  index++;
                  if ( (42 < j) && (j < 48) && (arrays[j][k] < 0.7) ){
                    SOF += arrays[j][k];
                    Fcnt++;
                  }
                  if ( (222 < j) && (j < 228) && (arrays[j][k] < 0.7) ) {
                    SOB += arrays[j][k];
                    Bcnt++;
                  }

                  if ((arrays[j][k] != 0) && (arrays[j][k] < 0.9)) {
                      angle_cnt[j/45] = true;
                      cnt_a[j/45] = true;
                  }
              }
            SOF /= Fcnt;
            SOB /= Bcnt;
          }
          //data input code end


         // close warning code start
          int wig = 0;
          bool FL_warning = false;
          bool FR_warning = false;
          bool RF_warning = false;
          bool RB_warning = false;
          bool BR_warning = false;
          bool BL_warning = false;
          bool LB_warning = false;
          bool LF_warning = false;
          const float THRESHOLD = 1.5;
          //const int WIG_TIME = 1500;
          //const int WIG_ANGLE = 30;
          for(int a = 0; a < 8; a++) {
            if ((angle_cnt[a] == true) && (cnt_a[a] == true)) {
              switch(a) {
                case 0:
                    FL_warning = true;
                    cout << "\n\n\nFL WARNING!\n\n\n" << endl;
                    break;
                case 1:
                    FR_warning = true;
                    cout << "\n\n\nFR WARNING!\n\n\n" << endl;
                    break;
                case 2:
                    RF_warning = true;
                    cout << "\n\n\nRF WARNING!\n\n\n" << endl;
                    break;
                case 3:
                    RB_warning = true;
                    cout << "\n\n\nRB WARNING!\n\n\n" << endl;
                    break;
                case 4:
                    BR_warning = true;
                    cout << "\n\n\nBR WARNING!\n\n\n" << endl;
                    break;
                case 5:
                    BL_warning = true;
                    cout << "\n\n\nBL WARNING!\n\n\n" << endl;
                    break;
                case 6:
                    LB_warning = true;
                    cout << "\n\n\nLB WARNING!\n\n\n" << endl;
                    break;
                case 7:
                    LF_warning = true;
                    cout << "\n\n\nLF WARNING!\n\n\n" << endl;
                    break;
                default:
                    break;
              }
            }
          }

     if (FL_warning && FR_warning && BL_warning && BR_warning && RF_warnin>
        // 모든 방향이 막혔을 때
        motor.Call(0);
      }
      else if (FL_warning && FR_warning && RF_warning && RB_warning && LF_w>
        // 전방 세 방향이 막혔을 때
        motor.Call(0);
      }
      else if (FR_warning && FL_warning) {
        //전방이 막혔을 때
        motor.Call(0);
      }
      else if (RF_warning) {
        // 전방 오른쪽 방향이 막혔을 때
        motor.Call(4);
      }
      else if (LF_warning) {
        // 전방 왼쪽 방향이 막혔을 때
        motor.Call(3);
      }
      /*else if (RF_warning && RB_warning) {
        // 우측방향이 막혔을 때
        motor.Call(4);
      }
      else if (LF_warning && LB_warning) {
        // 좌측방향이 막혔을 때
        motor.Call(3);
      }*/
      else if (FR_warning) {
        motor.Call(4);
      }
     else if (FL_warning) {
        motor.Call(3);
      }
      else {
        // 모든 방향이 자유로울 때
        motor.Call(1);
      }
      // close warning code end


      laser_pub->publish(*scan_msg);
    }
    else {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }
    if (!rclcpp::ok()) {
      break;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();

  return 0;
}
