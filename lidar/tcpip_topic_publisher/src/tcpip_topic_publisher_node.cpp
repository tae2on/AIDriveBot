#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"

#define SERVER_IP "192.168.0.25"
#define SERVER_PORT 12346

int main(int argc, char** argv) {
    ros::init(argc, argv, "tcpip_topic_publisher");
    ros::NodeHandle nh;

    int sock = 0;
    struct sockaddr_in serv_addr;

    //소켓 생성
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation error");
        return -1;
    }

    memset(&serv_addr, '0', sizeof(serv_addr)); // serv_addr 구조체 초기화

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(SERVER_PORT);

    //서버 주소 설정
    if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr) <= 0) {
        perror("Invalid address/ Address not supported");
        return -1;
    }

    //서버에 연결
    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection failed");
        return -1;
    }

    ros::Rate rate(10);

    while (ros::ok()) {
        std::string message;
        std_msgs::String msg;
        std::string send_message;

        char buffer[1024] = {0};
        int valread = read(sock, buffer, 1024);

        //서버로부터 받은 데이터를 읽고 ROS토픽으로 발행
        if(valread > 0) {
            try{        //규격 에러로 인한 예외처리
               message = buffer;
               msg.data = message;
               pub.publish(msg);
               ROS_INFO("Receive message: %s", msg.data.c_str());
           } catch(const std::exception& e) {
               ROS_ERROR("Error decoding message: %s", e.what());
           }

        }

        //입력받은 메시지를 서버로 전송
        std::cout << "Enter a message to send: ";
        std::getline(std::cin, send_message);

        if (send(sock, send_message.c_str(), send_message.size(), 0) < 0) {
            perror("Send failed");
            return -1;
        }

        ros::spinOnce();
        rate.sleep();
    }

    close(sock);

    return 0;
}
