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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

//#include <thread>

#include <fstream>
#include <iostream>
#include <thread>

using namespace std;

int s, i;
int nbytes;
struct sockaddr_can addr;
struct ifreq ifr;
struct can_frame frame;
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
//float alt = 0.0;
uint8_t gps_ststus = 0;

bool hed_up = false;
bool pos_up = false;
bool acc_xy_up = false,acc_z_up = false, acc_up = false;
bool gyr_xy_up = false, gyr_z_up = false, gyr_up = false;
bool alt_up = false;

float gravity = 9.8;
float deg2rad = 3.14159265359/180;

bool write_file = true;

/**
 * This tutorial demonstrates simple transmitting CAN messages over the ROS system.
 */

int can_read(void)
{
    //printf("CAN Sockets Receive Demo\r\n");

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
		return 1;
	}

	strcpy(ifr.ifr_name, "can0" );
	ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("Bind");
            return 1;
        }

    // printf("debug/n");

    while (true)
    {
        nbytes = read(s, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            perror("Read");
            return 1;
        }

        // printf("0x%03X [%d] ",frame.can_id, frame.can_dlc);

        switch (frame.can_id)
        {
            case 0x10b:
                {
                    uint16_t heading = frame.data[0] | frame.data[1]<<8;
                    int16_t pitch = frame.data[2] | frame.data[3]<<8;
                    int16_t roll = frame.data[4] | frame.data[5]<<8;

                    ang[0] = float(heading/100.0);
                    ang[1] = float(pitch/100.0);
                    ang[2] = float(roll/100.0);

                    hed_up = true;

                    break;
                }

            case 0x20b:
                {
                    int32_t lat = frame.data[0] | frame.data[1]<<8 | frame.data[2]<<16 | frame.data[3]<<24;
                    int32_t lon = frame.data[4] | frame.data[5]<<8 | frame.data[6]<<16 | frame.data[7]<<24;

                    pos[0] = double(lat/1e7);//double(lat/1e7-90.0);
                    pos[1] = double(lon/1e7);//double(lon/1e7-180.0);

                    pos_up = true;

                    break;
                }

            case 0x30b:
                {
                    int32_t altitude_tem = frame.data[0] | frame.data[1]<<8 | frame.data[2]<<16 | frame.data[3]<<24;
                    uint8_t baseline = frame.data[4];

                    altitude.data = float(altitude_tem/1000.0);

                    alt_up = true;

                    break;
                }

            case 0x60b:
                {
                    int32_t gyr_x = frame.data[0] | frame.data[1]<<8 | frame.data[2]<<16 | frame.data[3]<<24;
                    int32_t gyr_y = frame.data[4] | frame.data[5]<<8 | frame.data[6]<<16 | frame.data[7]<<24;

                    gyr[0] = float(gyr_x/100000.0)*deg2rad;
                    gyr[1] = float(gyr_y/100000.0)*deg2rad;

                    gyr_xy_up = true;

                    break;
                }

            case 0x70b:
                {
                    int32_t gyr_z = frame.data[0] | frame.data[1]<<8 | frame.data[2]<<16 | frame.data[3]<<24;
		    int32_t acc_z = frame.data[4] | frame.data[5]<<8 | frame.data[6]<<16 | frame.data[7]<<24;

                    gyr[2] = float(gyr_z/100000.0)*deg2rad;
		    acc[2] = float(acc_z/100000.0);

                    gyr_z_up = true;
		    acc_z_up = true;

                    break;
                }

            case 0x50b:
                {
                    // uint32_t acc_x = frame.data[0] | frame.data[1]<<8 | (frame.data[2] & 0xF0)<<12;
                    // uint32_t acc_y = (frame.data[2] & 0x0F) | frame.data[3]<<4 | frame.data[4]<<12;
                    // uint32_t acc_z = frame.data[5] | frame.data[6]<<8 | (frame.data[7] & 0xF0)<<12;

                    int32_t acc_x = frame.data[0] | frame.data[1]<<8 | frame.data[2]<<16 | frame.data[3]<<24;
                    int32_t acc_y = frame.data[4] | frame.data[5]<<8 | frame.data[6]<<16 | frame.data[7]<<24;
                    //int32_t acc_z = frame.data[5] | frame.data[6]<<8 | (frame.data[7] & 0x0F)<<16;

                    acc[0] = float(acc_x/100000.0);
                    acc[1] = float(acc_y/100000.0);
                    //acc[2] = float(acc_z/10000.0-50.0)*gravity;


                    acc_xy_up = true;

                    break;
                }
                
            case 0x31b:
                {
                    gps_ststus = frame.data[4] & 0x0F;
                }

            default:
                break;
        }
        if (gyr_xy_up && gyr_z_up)
        {
            gyr_xy_up = false;
            gyr_z_up = false;

            gyr_up = true;
        }
	if (acc_xy_up && acc_z_up)
        {
            acc_xy_up = false;
            acc_z_up = false;

            acc_up = true;
        }
    }

    if (close(s) < 0) {
        perror("Close");
        return 1;
    }

}

int main(int argc, char **argv)
{
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */

    ofstream outfile;
    outfile.open("position.dat");

    ofstream accfile;
    accfile.open("acceleration.dat");

    ros::init(argc, argv, "msg_transmit");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    ros::Publisher GNSSmsg_pub = n.advertise<std_msgs::Float64MultiArray>("gnss_msg", 3);
    ros::Publisher ACCEmsg_pub = n.advertise<std_msgs::Float32MultiArray>("acc_msg", 3);
    ros::Publisher GYROmsg_pub = n.advertise<std_msgs::Float32MultiArray>("gyr_msg", 3);
    ros::Publisher ANGLmsg_pub = n.advertise<std_msgs::Float32MultiArray>("ang_msg", 3);
    ros::Publisher ALTImsg_pub = n.advertise<std_msgs::Float32>("alt_msg", 3);

    ros::Publisher GPSmsg_pub = n.advertise<dbw::GPSFix>("gpsfix_msg", 3);
    ros::Publisher IMUmsg_pub = n.advertise<sensor_msgs::Imu>("imu_msg", 3);

    ros::Publisher NAVImsg_pub = n.advertise<sensor_msgs::NavSatFix>("nav_msg", 3);

    thread CANread(can_read);

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        posArray.data.clear();
        accArray.data.clear();
        gyrArray.data.clear();
        angArray.data.clear();

        //printf("0x%03X [%d] ",frame.can_id, frame.can_dlc);

        // can_read();

        // printf("0x%03X [%d] ",frame.can_id, frame.can_dlc);

        // for (i = 0; i < frame.can_dlc; i++)
        // printf("%02X ",frame.data[i]);

        // printf("\r\n");



        if (pos_up && hed_up && acc_up && gyr_up && alt_up)
        {
            pos_up = false;
            hed_up = false;
            acc_up = false;
            gyr_up = false;
            alt_up = false;

            /** Heading/Pitch/Roll **/
            for (size_t i = 0; i < 3; i++)
            {
                angArray.data.push_back(ang[i]);
            }

            ROS_INFO("heading: %0.4f, pitch: %0.4f, roll: %0.4f;",ang[0], ang[1], ang[2]);
            ANGLmsg_pub.publish(angArray);

            /** Position **/
            for (size_t i = 0; i < 2; i++)
            {
                posArray.data.push_back(pos[i]);
            }

            ROS_INFO("lat: %0.7f, lon: %0.7f;",pos[0], pos[1]);
            GNSSmsg_pub.publish(posArray);

            if (write_file)
            {
                // write inputted data into the file.
                outfile << std::fixed << std::setprecision(9) << pos[0] ;
                outfile << ";    ";
                outfile << std::fixed << std::setprecision(9) << pos[1]<< endl;
            }
            
            /** Altitude / 0.01 m **/
            ROS_INFO("altitude: %f;",altitude.data);

						ALTImsg_pub.publish(altitude);

            /** Angular rate / rad/s (raw 0.0001 deg/s) **/
            for (size_t i = 0; i < 3; i++)
            {
                gyrArray.data.push_back(gyr[i]);
            }

            ROS_INFO("gyr_x: %0.4f, gyr_y: %0.4f, gyr_z: %0.4f;",gyr[0], gyr[1], gyr[2]);

            GYROmsg_pub.publish(gyrArray);

            /** Acceleration / m/s (raw 0.0001 g) **/
            for (size_t i = 0; i < 3; i++)
            {
                accArray.data.push_back(acc[i]);
            }

            ROS_INFO("acc_x: %0.4f, acc_y: %0.4f, acc_z: %0.4f;",acc[0], acc[1], acc[2]);

            ACCEmsg_pub.publish(accArray);

            gpsfix.header.frame_id = "gpsfix";
            gpsfix.header.stamp = ros::Time::now();

            gpsfix.latitude = pos[0];
            gpsfix.longitude = pos[1];
            gpsfix.altitude = altitude.data;

            gpsfix.track = ang[0];
            gpsfix.pitch = ang[1];
            gpsfix.roll = ang[2];

            GPSmsg_pub.publish(gpsfix);

            //calculate orientation
            double yaw = ang[0]*deg2rad;
            double pitch_rad = ang[1]*deg2rad;
            double roll_rad = ang[2]*deg2rad;
            double cy = cos(yaw * 0.5);
            double sy = sin(yaw * 0.5);
            double cp = cos(pitch_rad * 0.5);
            double sp = sin(pitch_rad * 0.5);
            double cr = cos(roll_rad * 0.5);
            double sr = sin(roll_rad * 0.5);
            imumsg.header.frame_id = "imu";
            imumsg.header.stamp = ros::Time::now();
            imumsg.linear_acceleration.x = acc[0];
            imumsg.linear_acceleration.y = acc[1];
            imumsg.linear_acceleration.z = acc[2];
            imumsg.angular_velocity.x = gyr[0];
            imumsg.angular_velocity.y = gyr[1];
            imumsg.angular_velocity.z = gyr[2];

            imumsg.orientation.w = cr * cp * cy + sr * sp * sy;
            imumsg.orientation.x = sr * cp * cy - cr * sp * sy;
            imumsg.orientation.y = cr * sp * cy + sr * cp * sy;
            imumsg.orientation.z = cr * cp * sy - sr * sp * cy;

            IMUmsg_pub.publish(imumsg); 

            navMsg.header.frame_id = "gps_navfix";
            navMsg.header.stamp = ros::Time::now();

            navStatus.status = gps_ststus;

            navMsg.status.status = navStatus.status;
            navMsg.status.service = navStatus.SERVICE_GPS;

            navMsg.latitude = pos[0];
            navMsg.longitude = pos[1];
            navMsg.altitude = altitude.data;

            NAVImsg_pub.publish(navMsg); 

            if (write_file)
            {
                accfile << std::fixed << std::setprecision(4) << acc[0];
                accfile << ";    ";
                accfile << std::fixed << std::setprecision(4) << acc[1];
                accfile << ";    ";
                accfile << std::fixed << std::setprecision(4) << acc[2] << endl;
            }

            ros::spinOnce();
            loop_rate.sleep();
            //ros::Duration(1).sleep();
            //usleep(1000000);
        }
        //ros::spinOnce();
        //loop_rate.sleep();
    }
}
