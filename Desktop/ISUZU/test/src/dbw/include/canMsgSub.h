#ifndef CANMSGSUB_H
#define CANMSGSUB_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/NavSatStatus.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "dbw/FloatArray.h"
#include "dbw/GPSFix.h"
#include "dbw/GPSStatus.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include <thread>
#include <canMsg.h>

class canMsgSub:public canMsg {
    public: 
    canMsgSub();
    void readGpsImudata();
    void readSpeed();
    void run();
    
    private:
    ros::Publisher GNSSmsg_pub;
    ros::Publisher ACCEmsg_pub;
    ros::Publisher GYROmsg_pub;
    ros::Publisher ANGLmsg_pub;
    ros::Publisher ALTImsg_pub;
    ros::Publisher GPSmsg_pub;
    ros::Publisher IMUmsg_pub;
    ros::Publisher NAVImsg_pub;
    ros::Publisher Speed_pub;
    ros::NodeHandle n;
    std_msgs::Float32 vehicle_speed;
    std_msgs::Float64MultiArray posArray;
    std_msgs::Float32MultiArray accArray;
    std_msgs::Float32MultiArray gyrArray;
    std_msgs::Float32MultiArray angArray;
    std_msgs::Float32 altitude;
    dbw::FloatArray NumFloat;
    dbw::GPSFix gpsfix;
    sensor_msgs::Imu imumsg;
    sensor_msgs::NavSatFix navMsg;
    sensor_msgs::NavSatStatus navStatus;
    double pos[2] = {0.0, 0.0};
    float acc[3] = {0.0, 0.0, 0.0};
    float gyr[3] = {0.0, 0.0, 0.0};
    float ang[3] = {0.0, 0.0, 0.0};
    uint8_t gps_ststus = 0;
    bool hed_up = false;
    bool pos_up = false;
    bool acc_xy_up = false,acc_z_up = false, acc_up = false;
    bool gyr_xy_up = false, gyr_z_up = false, gyr_up = false;
    bool alt_up = false;
    //float gravity = 9.8;
    float deg2rad = 3.14159265359/180;
    bool write_file = true;
};

#endif //CANMSGSUB_H
