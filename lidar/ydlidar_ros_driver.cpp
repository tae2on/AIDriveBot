#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
//#include "ydlidar_ros_driver/LaserFan.h"
#include "std_srvs/Empty.h"
#include "src/CYdLidar.h"
#include "std_msgs/String.h"
#include "ydlidar_config.h"
#include <limits>       // std::numeric_limits
#include <iostream>
#include <array>
#include <cmath>

using namespace std;

#define SDKROSVerision "1.0.2"
#define NUM  500
#define PI 3.14159256358979323846
#define V_max 46.98


CYdLidar laser;

bool stop_scan(std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res) {
    ROS_DEBUG("Stop scan");
    return laser.turnOff();
}

bool start_scan(std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res) {
    ROS_DEBUG("Start scan");
    return laser.turnOn();
}

/*int mypartition (float arr[], int low, int high) {
    float pivot = arr[high];    // pivot
    int i = (low - 1);  // Index of smaller element

    for (int j = low; j <= high- 1; j++) {
        // If current element is smaller than the pivot
        if (arr[j] > pivot) {
            i++;    // increment index of smaller element
            std::swap(arr[i], arr[j]);
        }
    }
    std::swap(arr[i + 1], arr[high]);
    return (i + 1);
}*/

/*void quickSort(float arr[], int low, int high) {
    if (low < high) {
        //pi is partitioning index, arr[p] is now
        //at right place 
        int pi = mypartition(arr, low, high);

        // Separately sort elements before
        // partition and after partition
        quickSort(arr, low, pi - 1);
        quickSort(arr, pi + 1, high);
    }
}*/

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    std::string goal_point = msg->data;
    ROS_INFO("Goal point is: %s", goal_point.c_str());
}

// This function takes last element as pivot, places
// the pivot element at its correct position in sorted
// array, and places all smaller (smaller than pivot)
// to left of pivot and all greater elements to right
// of pivot

int main(int argc, char** argv) {
    ros::init(argc, argv, "ydlidar_ros_driver");
    ROS_INFO("YDLIDAR ROS Driver Version: %s", SDKROSVerision);
    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
    ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud>("point_cloud",
        1);
    //  ros::Publisher laser_fan_pub =
    //    nh.advertise<ydlidar_ros_driver::LaserFan>("laser_fan", 1);

    ros::NodeHandle nh_private("~");
    std::string str_optvalue = "/dev/ydlidar";
    nh_private.param<std::string>("port", str_optvalue, "/dev/ydlidar");
    ///lidar port
    laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(),
        str_optvalue.size());

    ///ignore array
    nh_private.param<std::string>("ignore_array", str_optvalue, "");
    laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(),
        str_optvalue.size());

    std::string frame_id = "laser_frame";
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("server_messages", 50, chatterCallback);

    //////////////////////int property/////////////////
    /// lidar baudrate
    int optval = 230400;
    nh_private.param<int>("baudrate", optval, 230400);
    laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
    /// tof lidar
    optval = TYPE_TRIANGLE;
    nh_private.param<int>("lidar_type", optval, TYPE_TRIANGLE);
    laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
    /// device type
    optval = YDLIDAR_TYPE_SERIAL;
    nh_private.param<int>("device_type", optval, YDLIDAR_TYPE_SERIAL);
    laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
    /// sample rate
    optval = 9;
    nh_private.param<int>("sample_rate", optval, 9);
    laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
    /// abnormal count
    optval = 4;
    nh_private.param<int>("abnormal_check_count", optval, 4);
    laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
    //intensity bit count
    optval = 10;
    nh_private.param<int>("intensity_bit", optval, 10);
    laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

    //////////////////////bool property/////////////////
    /// fixed angle resolution
    bool b_optvalue = false;
    nh_private.param<bool>("resolution_fixed", b_optvalue, true);
    laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
    /// rotate 180
    nh_private.param<bool>("reversion", b_optvalue, true);
    laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
    /// Counterclockwise
    nh_private.param<bool>("inverted", b_optvalue, true);
    laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
    b_optvalue = true;
    nh_private.param<bool>("auto_reconnect", b_optvalue, true);
    laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
    /// one-way communication
    b_optvalue = false;
    nh_private.param<bool>("isSingleChannel", b_optvalue, false);
    laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
    /// intensity
    b_optvalue = false;
    nh_private.param<bool>("intensity", b_optvalue, false);
    laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
    /// Motor DTR
    b_optvalue = false;
    nh_private.param<bool>("support_motor_dtr", b_optvalue, false);
    laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

    //////////////////////float property/////////////////
    /// unit: °
    float f_optvalue = 180.0f;
    nh_private.param<float>("angle_max", f_optvalue, 180.f);
    laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
    f_optvalue = -180.0f;
    nh_private.param<float>("angle_min", f_optvalue, -180.f);
    laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
    /// unit: m
    f_optvalue = 16.f;
    nh_private.param<float>("range_max", f_optvalue, 16.f);
    laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
    f_optvalue = 0.1f;
    nh_private.param<float>("range_min", f_optvalue, 0.1f);
    laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
    /// unit: Hz
    f_optvalue = 10.f;
    nh_private.param<float>("frequency", f_optvalue, 10.f);
    laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

    bool invalid_range_is_inf = false;
    nh_private.param<bool>("invalid_range_is_inf", invalid_range_is_inf,
        invalid_range_is_inf);

    bool point_cloud_preservative = false;
    nh_private.param<bool>("point_cloud_preservative", point_cloud_preservative,
        point_cloud_preservative);

    ros::ServiceServer stop_scan_service = nh.advertiseService("stop_scan",
        stop_scan);
    ros::ServiceServer start_scan_service = nh.advertiseService("start_scan",
        start_scan);

    // initialize SDK and LiDAR
    bool ret = laser.initialize();

    if (ret) {//success
        //Start the device scanning routine which runs on a separate thread and enable motor.
        ret = laser.turnOn();
    }
    else {
        ROS_ERROR("%s\n", laser.DescribeError());
    }

    ros::Rate r(30);

    float S_max = 8.0;
    float S_min = 0.12;
    float FRmat[NUM] = {0};
    float AVGDD[NUM] = {0};
    float DDmat[NUM] = {0};
    float ANGmat[NUM] = {0};
    float FRvec_x[NUM] = {0};
    float FRvec_y[NUM] = {0};
    float Distance;
    float FAng;
    float Fr, Fa;
    float C0 = V_max;
    const int C1 = 0;
    const int Tf = 1;
    float Ds = (V_max * Tf) / 2;
    float Ka = V_max / (Ds * Ds * Ds);
    float C2 = 3 * (0 - V_max) / (S_max * S_max);
    float C3 = -2 * (0 - V_max) / (S_max * S_max * S_max);
    float angle = 0;
    float FAvec_x, FAvec_y;
    float TotVec_x = 0, TotVec_y = 0;
    float Next_Ang, FAdeg_Ang;
    float Pre_x = 0, Pre_y = 0, goal_x = 0, goal_y = 0;
    float Next_x, Next_y;
    float Next_degang, Next_dist;
    float tr_point_x[NUM] = { 0 };
    float tr_point_y[NUM] = { 0 };

    while (ret && ros::ok()) {
        LaserScan scan;

        if (laser.doProcessSimple(scan)) {
            sensor_msgs::LaserScan scan_msg;
            sensor_msgs::PointCloud pc_msg;
            //      ydlidar_ros_driver::LaserFan fan;
            ros::Time start_scan_time;
            start_scan_time.sec = scan.stamp / 1000000000ul;
            start_scan_time.nsec = scan.stamp % 1000000000ul;
            scan_msg.header.stamp = start_scan_time;
            scan_msg.header.frame_id = frame_id;
            pc_msg.header = scan_msg.header;
            //      fan.header = scan_msg.header;
            scan_msg.angle_min = (scan.config.min_angle);
            scan_msg.angle_max = (scan.config.max_angle);
            scan_msg.angle_increment = (scan.config.angle_increment);
            scan_msg.scan_time = scan.config.scan_time;
            scan_msg.time_increment = scan.config.time_increment;
            scan_msg.range_min = (scan.config.min_range);
            scan_msg.range_max = (scan.config.max_range);
            //      fan.angle_min = (scan.config.min_angle);
            //      fan.angle_max = (scan.config.max_angle);
            //      fan.scan_time = scan.config.scan_time;
            //      fan.time_increment = scan.config.time_increment;
            //      fan.range_min = (scan.config.min_range);
            //      fan.range_max = (scan.config.max_range);

            int size = (scan.config.max_angle - scan.config.min_angle) /
                scan.config.angle_increment + 1;
            scan_msg.ranges.resize(size,
                invalid_range_is_inf ? std::numeric_limits<float>::infinity() : 0.0);
            scan_msg.intensities.resize(size);
            pc_msg.channels.resize(2);
            int idx_intensity = 0;
            pc_msg.channels[idx_intensity].name = "intensities";
            int idx_timestamp = 1;
            pc_msg.channels[idx_timestamp].name = "stamps";

            for (size_t i = 0; i < scan.points.size(); i++) {
                int index = std::ceil((scan.points[i].angle - scan.config.min_angle) /
                    scan.config.angle_increment);

                if (index >= 0 && index < size) {
                    if (scan.points[i].range >= scan.config.min_range) {
                        scan_msg.ranges[index] = scan.points[i].range;
                        scan_msg.intensities[index] = scan.points[i].intensity;
                    }
                }

                if (point_cloud_preservative ||
                    (scan.points[i].range >= scan.config.min_range &&
                        scan.points[i].range <= scan.config.max_range)) {
                    geometry_msgs::Point32 point;
                    point.x = scan.points[i].range * cos(scan.points[i].angle);
                    tr_point_x[i] = point.x;
                    point.y = scan.points[i].range * sin(scan.points[i].angle);
                    tr_point_y[i] = point.y;
                    point.z = 0.0;
                    pc_msg.points.push_back(point);
                    pc_msg.channels[idx_intensity].values.push_back(scan.points[i].intensity);
                    pc_msg.channels[idx_timestamp].values.push_back(i * scan.config.time_increment);
                }

                //pointxy bubble sorting
                //quickSort(tr_point_x, 0, NUM-1);
                //goal_x = tr_point_x[0];
                //quickSort(tr_point_y, 0, NUM-1);
                //goal_y = tr_point_y[0];

                //        fan.angles.push_back(scan.points[i].angle);
                //        fan.ranges.push_back(scan.points[i].range);
                //        fan.intensities.push_back(scan.points[i].intensity);
            }

            //Potential Field=========================================================

            //input data to array, generate Attractive
            for (int i = 0; i < 500; i++) {
                float distance = scan.points[i].range;    //라이다센서에서 넘어오는 500여개의 데이터
                angle += 0.72;    //라이다 구동 Hz에 따른 각도값(오차 무시)

                if ((angle <= 360) && (distance < 8.00) && (scan.points[i].intensity >= 0)) {    //360도 내, 8미터 내 일때 거리값 입력하여 정렬
                    Distance = (sqrt((Pre_x - goal_x) * (Pre_x - goal_x) + (Pre_y - goal_y) * (Pre_y - goal_y)));    //이동한 거리계산
                    if ((distance == 0)) {        //거리가 0 으로 나오는 각도는 이전 각도로 저장
                        DDmat[i] = DDmat[i-1];
                    } else {
                        DDmat[i] = distance;
                    }
                    ANGmat[i] = angle;
                    FAng = atan2((goal_x - Pre_x), (goal_x - Pre_y));
                    FAdeg_Ang = FAng * 180 / PI;
                    FAvec_x = Distance * sin(FAng);    //유인력 x성분
                    FAvec_y = Distance * cos(FAng);    //유인력 y성분
                    if (Ds > Distance) {
                        Fa = V_max - Ka * pow((Ds - Distance), 3);    //유인력
                    } else {
                        Fa = V_max;
                    }
                }
            }

            //bubble sorting
            float temp1, temp2, temp3;
            for (int i = 0; i < 500; i++) {
                for (int j = i + 1; j < 500; j++) {
                    if (ANGmat[i] > ANGmat[j]) {
                        temp1 = ANGmat[i];
                        ANGmat[i] = ANGmat[j];
                        ANGmat[j] = temp1;
                        temp2 = DDmat[i];
                        DDmat[i] = DDmat[j];
                        DDmat[j] = temp2;
                    }
                }
            }

            //average DDmat    정렬된 거리중 튀는 거리값들 필터링
            for (int p = 0; p < 500; p++) {
                if (p == 0) {
                    AVGDD[0] = (DDmat[499] + DDmat[0] + DDmat[1]) / 3;
                } else if (p == 499) {
                    AVGDD[499] = (DDmat[498] + DDmat[499] + DDmat[0]) / 3;
                } else {
                    AVGDD[p] = (DDmat[p - 1] + DDmat[p] + DDmat[p + 1]) / 3;
                }
            }

            //generate Repersive
            for (int i = 0; i < 500; i++) {
                Fr = C0 + (C2 * AVGDD[i] / 1000 * AVGDD[i] / 1000) + (C3 * AVGDD[i] / 1000 * AVGDD[i] / 1000 * AVGDD[i] / 1000);
                FRmat[i] = Fr;
            }

            //bubble sorting
            for (int i = 0; i < 500; i++) {
                for (int j = i + 1; j < 500; j++) {
                    if (FRmat[i] > FRmat[j]) {
                        temp1 = FRmat[i];        //반발력 배열에서 낮은값 순으로 정렬 
                        FRmat[i] = FRmat[j];
                        FRmat[j] = temp1;
                        temp2 = AVGDD[i];         //낮은 순에 따라 필터링한 거리가 낮은 순 
                        AVGDD[i] = AVGDD[j];
                        AVGDD[j] = temp2;
                        temp3 = ANGmat[i];        //낮은 순에 따라 각도 내림차순
                        ANGmat[i] = ANGmat[j];
                        ANGmat[j] = temp3;
                    }
                }
            }

            //Repulsive vector
            for (int i = 0; i < 500; i++) {
                FRvec_x[i] = AVGDD[i] / 1000 * sin(ANGmat[i] * PI / 180);
                FRvec_y[i] = AVGDD[i] / 1000 * cos(ANGmat[i] * PI / 180);
            }
            for (int i = 0; i < 500; i++) {
                TotVec_x += FRvec_x[i];
                TotVec_y += FRvec_y[i];
            }

            // Calculate attractive force toward the goal
            float goal_distance = sqrt(pow(goal_x - Pre_x, 2) + pow(goal_y - Pre_y, 2));    //로봇의 현재위치와 목표위치 사이 거리 계산
            float goal_angle = atan2(goal_y - Pre_y, goal_x - Pre_x);                        //로봇의 현재 위치와 목표 위치 사이 각도 계산

            // Combine attractive and repulsive forces to get the total force vector
            float total_force_x = TotVec_x + C1 * cos(goal_angle);        //목표에 대한 인력과 전체 반발력 힘의 x성분 합
            float total_force_y = TotVec_y + C1 * sin(goal_angle);        //목표에 대한 인력과 전체 반발력 힘의 y성분 합

            // Calculate the angle and distance of the next move
            float next_angle, next_distance;
            if (total_force_x == 0 && total_force_y == 0) {        //로봇에 작용하는 힘 유무 판별
                // No force, maintain current direction
                next_angle = goal_angle;
                next_distance = goal_distance;
            } else {
                next_angle = atan2(total_force_y, total_force_x);        //합친 힘을 기반으로 다음 이동 각도 계산
                next_distance = sqrt(pow(total_force_x, 2) + pow(total_force_y, 2));    //합친 힘을 기반으로 다음 이동 거리 계산
            }

            cout << "Potential Field Vector: " << TotVec_x << ", " << TotVec_y << endl;

            float delta_x = next_distance * cos(next_angle);
            float delta_y = next_distance * sin(next_angle);
            Pre_x += delta_x;
            Pre_y += delta_y;

            Next_x = Pre_x; // Store the next x coordinate
            Next_y = Pre_y; // Store the next y coordinate
            Next_Ang = next_angle; // Store the next angle
            Next_degang = next_angle * 180 / PI; // Convert the angle to degrees
            Next_dist = next_distance; // Store the next distance


            //to move x,y angle
            /*Next_x = (TotVec_x + 50 * FAvec_x) / 260;
            Next_y = (TotVec_y + 50 * FAvec_y) / 260;
            if ((Next_x < 0) && (Next_y < 0)) {
                Next_Ang = -PI + atan(Next_x / Next_y);
            } else if ((Next_x >= 0) && (Next_y < 0)) {
                Next_Ang = PI + atan(Next_x / Next_y);
            } else {
                Next_Ang = atan(Next_x / Next_y);
            }
            Next_degang = Next_Ang * 180 / PI;
            Next_dist = sqrt(Next_x * Next_x + Next_y * Next_y);*/
            
            for(int i = 0; i < 500; i++) {
                cout << i << "   " << FAvec_x << "   " << FAvec_y << "   " << AVGDD[i] << "   " << ANGmat[i] << "   " << Next_x << ", " << Next_y;
                cout << "   " << Next_Ang << "   " << Next_degang << "   " << Next_dist << endl;

            }
            //Potential Field=========================================================

            scan_pub.publish(scan_msg);
            pc_pub.publish(pc_msg);
            //      laser_fan_pub.publish(fan);

        }
        else {
            ROS_ERROR("Failed to get Lidar Data");
        }

        r.sleep();
        ros::spinOnce();
    }

    laser.turnOff();
    ROS_INFO("[YDLIDAR INFO] Now YDLIDAR is stopping .......");
    laser.disconnecting();
    return 0;
}
