// Isuzu Technical Center of America


#include "ros/ros.h"
#include "std_msgs/String.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <iostream>       // std::cout
#include <thread>         // std::thread


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */


int16_t StrAng = 0;  // not in use currently .....
int8_t  StrAng_byte1 = 0;  // not in use currently .....
int8_t  StrAng_byte2 = 0;    // not in use currently .....

float target_speed = 0;
float vehicle_speed = 0;


bool brake_enable = 0;
bool torque_enable = 0;

float brake_demand_raw = 0; // range -15.687 ~ 15.687
uint16_t  brake_demand = ( brake_demand_raw + 15.687 ) * 2048; 
uint8_t brake_demand_byte1 = ( brake_demand & 0xff00 ) >> 8 ;
uint8_t brake_demand_byte2 =  brake_demand & 0xff;



int nbytes; // for read can bus



void chatterCallback(const std_msgs::Float64::ConstPtr& msg) 
{

	target_speed = msg->data;

}

void speed_control(){

	float speed_error = target_speed - vehicle_speed;
	
	if( speed_error < 0){

		torque_enable = 0;

		brake_enable = 1;

		int P_term = 2;
		brake_demand_raw = speed_error*P_term;
		if( brake_demand_raw < -10){
			brake_demand_raw = -10;
		}
		brake_demand = ( brake_demand_raw + 15.687 ) * 2048; 
		brake_demand_byte1 = ( brake_demand & 0xff00 ) >> 8 ;
		brake_demand_byte2 =  brake_demand & 0xff;
	}else if (speed_error >= 0) {
		// this section for torque control in future
		brake_enable = 0;		
	
	}

}

int speed_read(can_frame &frame, int &s){

	nbytes = read(s, &frame, sizeof(struct can_frame));

	if (nbytes < 0) {
	    perror("CAN Read Error ......");
	    return 1;
	}

	if (frame.can_id == (0x18fef100 | 0x80000000) ){     //read vehicle speed id
		
		uint16_t vehicle_speed_bytes = frame.data[2] | frame.data[1] << 8;
		vehicle_speed = vehicle_speed_bytes / 256.0;

	}
	
	return 0;
}


void thread_EBR(can_frame &frame, int &s, int &counter, int &checksum){

	// External Brake Request
	//+++++hard code++++++//
	//int a[16] = {0,1,2,3,4,5,7,8,9,10,11,12,13,14,15,0};

	frame.can_id = 0x0c040b2a | 0x80000000;
	frame.can_dlc = 8;
	frame.data[0] = brake_demand_byte1;//0x19;
	frame.data[1] = brake_demand_byte2;//0x2f;



	if (brake_enable){
		frame.data[2] = 0xe0;//XBR_control_mode = 2 maximum		
	}else{
		frame.data[2] = 0xc0;//XBR_control_mode = 0 override disable
	}



	frame.data[3] = 0x00;
	frame.data[4] = 0xff;
	frame.data[5] = 0xff;
	frame.data[6] = 0xff;

	checksum=frame.data[0]+frame.data[1]+frame.data[2]+frame.data[3]+frame.data[4]+frame.data[5]+frame.data[6]+(counter&0x0f)+0x2a+0x0b+0x04+0x0c;
	checksum=((checksum>>4)+checksum)&0x0f;

	//std::cout << "checksum = "<< checksum <<"\n";

	//checksum= a[counter];

	//std::cout << "checksum ="<< checksum <<"\n";

	/*
	//debug
	if(counter == 0){
		int A = frame.data[0]+frame.data[1]+frame.data[2]+frame.data[3]+frame.data[4]+frame.data[5]+frame.data[6] ;
		int B = counter&0x0f;
		int C = 0x2a+0x0b+0x04+0x0c;
		std::cout << "checksum A ="<< A <<"\n";
		std::cout << "checksum B ="<< B <<"\n";
		std::cout << "checksum C ="<< C <<"\n";
		std::cout << "A+B+C= ="<< A+B+C <<"\n";
		int D = A+B+C;		
		int E = (A+B+C)>>4;
		int F = (D+E)&0x0f;
		std::cout << "checksum = "<< D <<"\n";
	
	}	
	*/

	frame.data[7] = (checksum<<4) | counter;
	
	if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
		perror("EBR Write error1");
	}


	ROS_INFO("CAN ID: 0x%08X [%d] %02x %02x %02x %02x %02x %02x %02x %02x",frame.can_id, frame.can_dlc, frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);

}

void thread_ADR(can_frame &frame, int &s){

	// ACC Display Request
	frame.can_id = 0x18ff1211 | 0x80000000;
	frame.can_dlc = 8;
	frame.data[0] = 0xfd;
	frame.data[1] = 0xff;
	frame.data[2] = 0x01;
	frame.data[3] = 0xff; // signal: set_vehicle_speed_indication: 0:disable; 1~FA:enable with speed
	frame.data[4] = 0x00;

	if(brake_enable){
		frame.data[5] = 0xbf; // alarm on
	}else{
		frame.data[5] = 0x3f; // alarm off
	}

	frame.data[6] = 0x1f;// signal: (bit 6-8) acc_main_set
	frame.data[7] = 0x00;


	if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
		perror("Write error");
	}
	//std::cout << "CAN ID ADR is transmitting ...........................................\n";
	//ROS_INFO("CAN ID: 0x%08X [%d] %02x %02x %02x %02x %02x %02x %02x %02x",frame.can_id, frame.can_dlc, frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);

}

void thread_AD(can_frame &frame, int &s){

	// Transmit ACC Data
	frame.can_id = 0x18ff1311 | 0x80000000;
	frame.can_dlc = 8;
	frame.data[0] = 0xc8;
	frame.data[1] = 0x00;
	frame.data[2] = 0x7d;
	frame.data[3] = 0x7d;
	frame.data[4] = 0xfa;
	frame.data[5] = 0x4c; 
	frame.data[6] = 0x00;
	frame.data[7] = 0x00;


	if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
		perror("Write error");
	}

	//std::cout << "CAN ID AD is transmitting ...........................................\n";
	//ROS_INFO("CAN ID: 0x%08X [%d] %02x %02x %02x %02x %02x %02x %02x %02x",frame.can_id, frame.can_dlc, frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);

}

void thread_ACR2(can_frame &frame, int &s, int &counter){


	// Transmit ACR2 (acc control request)
	frame.can_id = 0x0cff5211 | 0x80000000;
	frame.can_dlc = 8;
	frame.data[0] = 0x14;//14:speed ctrl; 24: torque ctrl
	frame.data[1] = 0x00;//ff
	frame.data[2] = 0x00;//ff
	frame.data[3] = 0xfa;//ff //signal: targe_cruise_set_speed
	frame.data[4] = 0xff;//ff //signal: targe_cruise_set_speed
	frame.data[5] = 0x00;
	frame.data[6] = 0x10;//80
	frame.data[7] = counter; //this is a counter or not ??????????????????


	if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
		perror("Write error");
	}

	//std::cout << "CAN ID ACR2 is transmitting ................................... \n";
	//ROS_INFO("CAN ID: 0x%08X [%d] %02x %02x %02x %02x %02x %02x %02x %02x",frame.can_id, frame.can_dlc, frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);
}

void thread_LDWSD(can_frame &frame, int &s){

	// Transmit LDWS Data
	frame.can_id = 0x10ff5311 | 0x80000000;
	frame.can_dlc = 8;
	frame.data[0] = 0x00;
	frame.data[1] = 0x06;
	frame.data[2] = 0x0f;
	frame.data[3] = 0x0f;
	frame.data[4] = 0xfa; 
	frame.data[5] = 0xfa; 
	frame.data[6] = 0xff; //a0 not 20, see kenji-san reply
	frame.data[7] = 0xff;


	if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
		perror("Write error");
	}


	//std::cout << "CAN ID LDWSD is transmitting ................................... \n";
	//ROS_INFO("CAN ID: 0x%08X [%d] %02x %02x %02x %02x %02x %02x %02x %02x",frame.can_id, frame.can_dlc, frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);


}

void thread_CAMINFO(can_frame &frame, int &s){


	// Transmit  
	frame.can_id = 0x10FF6311 | 0x80000000;
	frame.can_dlc = 8;
	frame.data[0] = 0x00;
	frame.data[1] = 0x0c;//0c
	frame.data[2] = 0xff;//ff
	frame.data[3] = 0xff;//ff
	frame.data[4] = 0x7f;//7f 
	frame.data[5] = 0x3f;//07 
	frame.data[6] = 0xff;//ff
	frame.data[7] = 0xff;//ff


	if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
		perror("Write error");
	}


	//std::cout << "CAN ID CAMINFO is transmitting ................................... \n";
	//ROS_INFO("CAN ID: 0x%08X [%d] %02x %02x %02x %02x %02x %02x %02x %02x",frame.can_id, frame.can_dlc, frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);



}

void thread_TSC1_FACC(can_frame &frame, int &s, int &counter, int &checksum){

	// Transmit torque/speed control #1 (FACC)
	frame.can_id = 0x0C000011 | 0x80000000;
	frame.can_dlc = 8;
	if(torque_enable){
		frame.data[0] = 0xfe; // torque ctrl: enable:fe; disable:fc?
	}else{
		frame.data[0] = 0xfc; // torque ctrl: enable:fe; disable:fc?
	}
	frame.data[1] = 0x70; // 70 request_speed_limit_speed: 0-faff: 0-8031.875 rpm
	frame.data[2] = 0xff; // ff request_speed_limit_speed: 0-faff: 0-8031.875 rpm
	frame.data[3] = 0xa0; // a0 request_torque_limit: 0-fa: -125% - 125%
	frame.data[4] = 0xff;
	frame.data[5] = 0xf0;
	frame.data[6] = 0xff;
	checksum=frame.data[0]+frame.data[1]+frame.data[2]+frame.data[3]+frame.data[4]+frame.data[5]+frame.data[6]+(counter&0x0f)+0x11+0x00+0x00+0x0c;
	checksum=(((checksum>>6)&0x03)+(checksum>>3)+checksum)&0x07;
	frame.data[7] = (checksum<<4) | counter;


	if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
		perror("Write error");
	}


	//std::cout << "CAN ID TSC1_FACC is transmitting ................................... \n";
	//ROS_INFO("CAN ID: 0x%08X [%d] %02x %02x %02x %02x %02x %02x %02x %02x",frame.can_id, frame.can_dlc, frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);



}

void thread_CAMINFO2(can_frame &frame, int &s){

	// Transmit  
	frame.can_id = 0x18ffa111 | 0x80000000;
	frame.can_dlc = 8;
	frame.data[0] = 0x0f;
	frame.data[1] = 0x0f;
	frame.data[2] = 0xff;
	frame.data[3] = 0xa0;  // what's the logic? need to test various set speed? ...
	frame.data[4] = 0x3f;  
	frame.data[5] = 0xff;  
	frame.data[6] = 0xff;
	frame.data[7] = 0xff;


	if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
		perror("Write error");
	}

	//std::cout << "CAN ID CAMINFO2 is transmitting ................................... \n";
	//ROS_INFO("CAN ID: 0x%08X [%d] %02x %02x %02x %02x %02x %02x %02x %02x",frame.can_id, frame.can_dlc, frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);


}

void thread_AD2(can_frame &frame, int &s){


	// Transmit ACC Data2 
	frame.can_id = 0x18ff3511 | 0x80000000;
	frame.can_dlc = 8;
	frame.data[0] = 0x00;
	frame.data[1] = 0x00;
	frame.data[2] = 0x00;
	frame.data[3] = 0xff;
	frame.data[4] = 0xff; 
	frame.data[5] = 0xff; 
	frame.data[6] = 0xff;
	frame.data[7] = 0xff;


	if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
		perror("Write error");
	}

	//std::cout << "CAN ID AD2 is transmitting ................................... \n";
	//ROS_INFO("CAN ID: 0x%08X [%d] %02x %02x %02x %02x %02x %02x %02x %02x",frame.can_id, frame.can_dlc, frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);


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
	ros::Subscriber sub = n.subscribe("target_speed", 1000, chatterCallback); 

	/**
	* ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	*/
	//ros::spin();

	ros::Rate loop_rate(100); //Hz, 50Hz = 10ms


	// add threads

	//std::thread first (thread_EBR);   
	/*	
	std::thread second (thread_ADR);   
	std::thread third (thread_AD);   
	std::thread fouth (thread_ACR2);   
	std::thread fifth (thread_LDWSD);   
	std::thread sixth (thread_CAMINFO);   
	std::thread seventh (thread_TSC1_FACC);   
	std::thread eighth (thread_CAMINFO2);   
	std::thread ninth (thread_AD2);   
	*/




	struct sockaddr_can addr; // socketCAN
	struct ifreq ifr;
	struct can_frame frame;
	int s;



	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
	perror("Socket error");
	}
	strcpy(ifr.ifr_name, "can0" );
	ioctl(s, SIOCGIFINDEX, &ifr);
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
	perror("Bind error");
	}



	int i = 1;
	int counter_EBR = 0;
	int counter_ACR2 = 0;
	int counter_TSC1 = 0;
	int checksum_EBR;
	int checksum_TSC1;

	while (ros::ok())
	{

		ros::spinOnce();   // if there is not data transmitted from the topic, spinOnce() will take a long time! and the actual execution time will be longer than loop_rate()!

		std::cout << "Main loop number = " << i << " \n";


		// for 10ms messages

		if(i%1 == 0){

			// transmit ACR2

			thread_ACR2(frame, s, counter_ACR2);

			if(counter_ACR2 >= 255){
				counter_ACR2 = 0;		
			}else{
				counter_ACR2++;
			}


			// transmit CAMINFO

			thread_CAMINFO(frame, s);


			// transmit TSC1_FACC torque/speed control #1

			thread_TSC1_FACC(frame, s, counter_TSC1, checksum_TSC1);

			if(counter_TSC1 >= 7){
				counter_TSC1 = 0;		
			}else{
				counter_TSC1++;
			}




			// read vehicle speed
			speed_read(frame, s);


			// execute speed controller
			speed_control();







		}		




		// for 100 ms messages

		if(i%10 == 0){
		
			// transmit ADR

			thread_ADR(frame, s);


			// transmit AD

			thread_AD(frame, s);


			// transmit LDWSD

			thread_LDWSD(frame, s);


			
			// transmit CAMINFO2
			thread_CAMINFO2(frame, s);


			// transmit AD2

			thread_AD2(frame, s);
			

		}


		// for 20 ms messages

		if(i%2 == 0){


			// transmit EBR
		
			thread_EBR(frame, s, counter_EBR, checksum_EBR);

			if(counter_EBR >= 15){
				counter_EBR = 0;		
			}else{
				counter_EBR++;
			}

		}



		i++;




		
		loop_rate.sleep();




	}
	return 0;
}
