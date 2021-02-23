#include <ros/ros.h>
#include <iostream>
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
    std_msgs::Float64 StrAng;

    float input;

    ros::init(argc, argv, "angle");

    ros::NodeHandle n;

    ros::Publisher pub=n.advertise<std_msgs::Int16MultiArray>("targetspeed",10);

    ros::Rate loop_rate(100); //Hz, 10Hz = 100ms

    while(ros::ok()){

	StrAng.data.clear();
	
	std::cout<<"Enter the target speed:  ";	
	
	std::cin>>input;

        StrAng.data.push_back(input);

	//std::cout<<StrAng<<"\n";

	pub.publish(StrAng);

 	loop_rate.sleep();
    }
    return 0;
}
	
