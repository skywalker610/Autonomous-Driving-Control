#include <ros/ros.h>
#include <iostream>
#include "std_msgs/Int16MultiArray.h"

int main(int argc, char **argv)
{
    std_msgs::Int16MultiArray StrAng;

    int16_t input;

    ros::init(argc, argv, "angle");

    ros::NodeHandle n;

    ros::Publisher pub=n.advertise<std_msgs::Int16MultiArray>("torque_req",10);

    ros::Rate loop_rate(10); //Hz, 10Hz = 100ms

    while(ros::ok()){

	StrAng.data.clear();
	
	std::cout<<"Enter steering angle in the range -1440 ~ 1440:  ";	
	
	std::cin>>input;

        StrAng.data.push_back(input);

	//std::cout<<StrAng<<"\n";

	pub.publish(StrAng);

 	loop_rate.sleep();
    }
    return 0;
}
	
