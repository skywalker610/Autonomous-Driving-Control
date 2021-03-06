// Isuzu Technical Center of America


#include "ros/ros.h"
#include "std_msgs/String.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16MultiArray.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */


int16_t StrAng = 0;
int8_t  StrAng_byte1 = 0;
int8_t  StrAng_byte2 = 0;

void chatterCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{

	StrAng = msg->data[0]*10;
	StrAng_byte1 = (StrAng & 0xff00) >> 8;
	StrAng_byte2 = StrAng & 0xff;
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
	ros::init(argc, argv, "sbw_send");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;


	/*if (close(s) < 0) {
	perror("Close");
	}*/
	/**
	* The subscribe() call is how you tell ROS that you want to receive messages
	* on a given topic.  This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing.  Messages are passed to a callback function, here
	* called chatterCallback.  subscribe() returns a Subscriber object that you
	* must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	* object go out of scope, this callback will automatically be unsubscribed from
	* this topic.
	*
	* The second parameter to the subscribe() function is the size of the message
	* queue.  If messages are arriving faster than they are being processed, this
	* is the number of messages that will be buffered up before beginning to throw
	* away the oldest ones.
	*/
	ros::Subscriber sub = n.subscribe("str_wheel_ang_cmd", 1000, chatterCallback); 

	/**
	* ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	*/
	//ros::spin();


	ros::Rate loop_rate(50); //Hz, 50Hz = 20ms
	int counter = 0;
	while (ros::ok())
	{


		struct sockaddr_can addr;
		struct ifreq ifr;
		struct can_frame frame;



		//Initialization
		int s;
		if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
		}

		strcpy(ifr.ifr_name, "can0" );
		ioctl(s, SIOCGIFINDEX, &ifr);

		memset(&addr, 0, sizeof(addr));
		addr.can_family = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex;

		if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Bind");
		}


		if ( counter < 300 ){

			// 1st step: initial setting
			frame.can_id = 0x600;
			frame.can_dlc = 8;
			frame.data[0] = 0x00;//fa;
			frame.data[1] = 0x00;//ff;
			frame.data[2] = 0x00;
			frame.data[3] = 0x00;
			frame.data[4] = 0x00;
			frame.data[5] = 0x00;
			frame.data[6] = 0x00;
			frame.data[7] = 0x00;

			if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
			perror("Write");
			}

			ROS_INFO("Initial setting: 0x%08X [%d] %02x %02x %02x %02x %02x %02x %02x %02x",frame.can_id, frame.can_dlc, frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);

			frame.can_id = 0x603;
			frame.can_dlc = 8;
			frame.data[0] = 0x00;//fa;
			frame.data[1] = 0x00;//ff;
			frame.data[2] = 0x00;
			frame.data[3] = 0x00;
			frame.data[4] = 0x00;
			frame.data[5] = 0x00;
			frame.data[6] = 0x00;
			frame.data[7] = 0x00;

			if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
			perror("Write");
			}

			ROS_INFO("Initial setting: 0x%08X [%d] %02x %02x %02x %02x %02x %02x %02x %02x",frame.can_id, frame.can_dlc, frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);    

		} else if (counter >= 300 && counter < 600){
			// 3rd step: zero steer angle

			frame.can_id = 0x600;
			frame.can_dlc = 8;
			frame.data[0] = 0x00;
			frame.data[1] = 0x00;
			frame.data[2] = 0x00;
			frame.data[3] = 0x00;
			frame.data[4] = 0x00;
			frame.data[5] = 0x00;
			frame.data[6] = 0x01; // ANG0 = 1 (zero point enable)
			frame.data[7] = 0x00;


			if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
			perror("Write");
			}

			ROS_INFO("Please zero steer angle now ...: 0x%08X [%d] %02x %02x %02x %02x %02x %02x %02x %02x",frame.can_id, frame.can_dlc, frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);


		} else {

			// Transmit 0X603

			frame.can_id = 0x603;
			frame.can_dlc = 8;
			frame.data[0] = 0x00;
			frame.data[1] = 0x00;
			frame.data[2] = 0x00; // VSPD (vehicle speed)
			frame.data[3] = 0x00; // VSPD (vehicle speed)
			frame.data[4] = 0x01; // VSPDEN (vehicle speed enable)
			frame.data[5] = 0x00;
			frame.data[6] = 0x00; // ANG0 = 0 (zero point disable)
			frame.data[7] = 0x00;

			if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
			perror("Write");
			}

			ROS_INFO("Transmit speed: 0x%08X [%d] %02x %02x %02x %02x %02x %02x %02x %02x",frame.can_id, frame.can_dlc, frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);



			// Transmit 0x600: CTRLMODE, CTRLREQ, and ANGREQ / TRQREQ 
			frame.can_id = 0x600;
			frame.can_dlc = 8;
			frame.data[1] = 0x00;
			frame.data[0] = 0x00;
			frame.data[2] = StrAng_byte1; // Steer angle
			frame.data[3] = StrAng_byte2; // Steer angle
			frame.data[4] = 0x01; // CTRLREQ
			frame.data[5] = 0x01; // CTRLMODE = 1 (angle control mode)
			frame.data[6] = 0x00;
			frame.data[7] = 0x00;


			if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
				perror("Write");
			}

			ROS_INFO("Enable CTRLMODE and CTRLRQE: 0x%08X [%d] %02x %02x %02x %02x %02x %02x %02x %02x",frame.can_id, frame.can_dlc, frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);




			if (close(s) < 0) {
				perror("Close");
			}
		}

		counter++;

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
