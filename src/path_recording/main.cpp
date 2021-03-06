//1023 mx

#include <ros/ros.h>
#include <ros/package.h> // for ros::package::getPath()

#include "std_msgs/String.h" // for sending msg type: std_msgs/String
#include "std_msgs/Int16.h" // for sending msg type: std_msgs/String
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>  // for cout double with precision(6)
#include <cmath>


#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <netinet/in.h>



#include "boost/asio.hpp"
#include <boost/array.hpp>
#include <boost/bind.hpp>

#include <thread>

#include <math.h>
#include <chrono>
#include <vector>



using namespace std;


#include "std_msgs/Float64.h" //ROS

#include <visualization_msgs/Marker.h>//for publishing markers to rviz

#include <tf/transform_broadcaster.h> // for broadcasting a frame

#include <tf/transform_datatypes.h> // for publishing vehicle yaw using quaternion



double gnss_arr[2];
float hedg_arr[1];

bool pos_up = false;
bool ang_up = false;



float hedg2ENU (float hedg) // for simulation, range from -Pi to Pi
{
	hedg = 3.1415926f*2.0f - hedg/180.0f*3.1415926f;
	hedg = hedg - 3.1415926f*3.0f/2.0f;
	if ( hedg < -3.1415926f ){
		hedg = hedg + 2.0f*3.1415926f;
	}
	return hedg;
}




void gnssCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{

	int i = 0;


	for(std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	{
	        gnss_arr[i] = *it;
	        i++;
	}

	int j = 0;
	for(j = 0; j < 2; j++)
	{
		printf("%0.7f, ", gnss_arr[j]);
	}
	printf("\n");

	ROS_INFO("I heard gnss: [%0.7f, %0.7f]", gnss_arr[0],gnss_arr[1]);

	pos_up = true;

	return;
}

void hedgCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{

	int i = 0;


	for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	{
	        hedg_arr[i] = *it;
	        i++;
	}

	int j = 0;
	for(j = 0; j < 1; j++)
	{
		printf("%f, ", hedg_arr[j]);
	}
	printf("\n");

	ROS_INFO("I heard heading: [%f]", hedg_arr[0]);

	ang_up = true;

	return;
}

void GNSS_read()
{
	while(ros::ok())
	{
		ros::spinOnce();
	}
}



int main ( int argc, char**argv){

	//Initialize the ROS system.
	ros::init(argc, argv, "gps_recording");

	// Establish this program as a ROS node.
	ros::NodeHandle n;

	// Create publisher object
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_markers", 1000);

	ros::Publisher float_pub = n.advertise<std_msgs::Float64>("float", 1000);

	ros::Publisher str_pub = n.advertise<std_msgs::Int16>("steering_angle", 1000);


	ros::Subscriber gnss_sub = n.subscribe("gnss_msg", 1, gnssCallback);
	ros::Subscriber hedg_sub = n.subscribe("ang_msg", 1, hedgCallback);


	ros::spinOnce();

	// thread GNSSread(GNSS_read); //????????????????????

  	ros::Rate rate(30);

	//Send some output as a log message.
	ROS_INFO_STREAM("Hello, ROS!");

	// Broadcast a frame
	tf::TransformBroadcaster br;
	tf::Transform transform;


	//Parameters

	int i = 1;

	double lon, lat;
	float x0, y0, hedg, x0_pre, y0_pre; // vehicle ENU coordinate
	float x0_frt, y0_frt;// vehicle front head
	float d_frt = 3.0 ; // vehicle front head distance (m)

	double wps_lon[3000], wps_lat[3000];
	float wps_x_raw[3000], wps_y_raw[3000];

	double in_lat, in_lon;
	int lat_num;
	int lon_num;

	float wps_x[3000], wps_y[3000]; 
	int wps_num = 0;

	float wps_step;


	double lat_ref, lon_ref;
	int idx_glb, idx;
	//float x1, x2, y1, y2, x3, y3;
	double x1, x2, y1, y2, x3, y3, x4, y4;
	float ang1, ang2;
	float d_v2seg_min = 1000;
	float d_v2seg;

	int wps_idx; //global 0, 1, 2, ...


	float d_o2v; // circle center to vechile
	int Kp, Kd;
	float offset_ctrl;
	float offset_ctrl_pre = 0;
	float d_offset;
	float str_step_ref;
	float str_step;
	int str_cmd; // must be int
	float d_p2line;
	int side2line;

	float theta1, theta3, d_theta;// for plotting trajectory plan
	float traj_x[100], traj_y[100];


	float d_travel = 0.1;//simulation

	//double add_x0, add_y0, add_hedg; // simulation
	float add_x0, add_y0, add_hedg; // simulation
	float add_cx0, add_cy0, add_r;




	// print to user
	cout << "Hello World!" << endl;



	// Geodetic to ENU
	lon_ref = -83.5005484;
	lat_ref = 42.3842014;





	// Get GPS  =================================================================================


	while (pos_up != true && ang_up != true) // ensure callback function is receiving data
	{
		ros::spinOnce();
		
	}


	// while (lat == 0 || lon == 0) // ensure correct data received 
	// {
	// 	ros::spinOnce();
	// 	printf("lat = %0.7f\n", lat);
	// }
	// exit(0);

	ros::spinOnce(); // receiving updated data

	lat = gnss_arr[0];						
	lon = gnss_arr[1];						
	hedg = hedg_arr[0];	


	x0 = (lon - lon_ref)*82230.670304; //
	y0 = (lat - lat_ref)*111132.944444; // lon_lat difference transfer into distance difference
	hedg = hedg2ENU(hedg);// convert gnss orientation from 0, 360 to ENU orientation from -pi, pi
	x0_frt = x0 + d_frt*cos(hedg); 
	y0_frt = y0 + d_frt*sin(hedg);

	
	x0_pre = x0;
	y0_pre = y0;
	
	/*
	cout << "x0" << ":" << x0 << "\n";
	cout << "y0" << ":" << y0 << "\n";
	*/
	



	// While loop
	while(ros::ok()){

		cout<<"while loop: Value of variable i is: "<<i << "-------------------------------------------------------------------" <<endl;
		i++;
		// exit(0);

		// Publish waypoints and lables for rviz visualization==========================================================================



		// Get GPS


		pos_up = false;
		ang_up = false;


		while (pos_up != true && ang_up != true) // ensure callback function is receiving data
		{
			ros::spinOnce();
		}


		while (lat == 0 || lon == 0) // ensure correct data received 
		{
			ros::spinOnce();
		}

		ros::spinOnce(); // receiving updated data

		lat = gnss_arr[0];						
		lon = gnss_arr[1];						
		hedg = hedg_arr[0];	
		
		printf("lat = %0.7f\n", lat);
		printf("lon = %0.7f\n", lon);
		printf("heading = %f\n", hedg);


		x0 = (lon - lon_ref)*82230.670304;
		y0 = (lat - lat_ref)*111132.944444;
		hedg = hedg2ENU(hedg);// convert gnss orientation from 0, 360 to ENU orientation from -pi, pi

		printf("x0: %f, y0: %f\n",x0, y0);


		x0_frt = x0 + d_frt*cos(hedg); 
		y0_frt = y0 + d_frt*sin(hedg);

		wps_step = (x0 - x0_pre)*(x0 - x0_pre) + (y0 - y0_pre)*(y0 - y0_pre);

		//exit(0); // debug -------------------------------------------------



		if(i==1){
			ofstream write_lon;
			write_lon.open("lon_raw.txt"); // to clear data in lon_raw.txt			
			write_lon.close();

			ofstream write_lat;
			write_lat.open("lat_raw.txt"); // to clear data in lat_raw.txt	
			write_lat.close();
		}



		if(wps_step >= 3){
			ofstream write_lon;
			write_lon.open("lon_raw.txt", ios::app); // adding ios:app will write data below the last line
			write_lon << std::fixed << std::setprecision(7) << lon << endl;
			write_lon.close();

			ofstream write_lat;
			write_lat.open("lat_raw.txt", ios::app); // adding ios:app will write data below the last line
			write_lat << std::fixed << std::setprecision(7) << lat << endl;
			write_lat.close();

			x0_pre = x0;
			y0_pre = y0;


			// publish vehicle's coordinates using visualization_msgs::Marker::POINTS
			visualization_msgs::Marker points;
			points.header.frame_id = "frame";
			points.header.stamp = ros::Time::now();
			points.ns = "waypoints";
			points.action = visualization_msgs::Marker::ADD;
			points.pose.orientation.w = 1.0;
			points.id = i; // %Tag(ID)%
			points.type = visualization_msgs::Marker::POINTS; 
			points.scale.x = 2;// %Tag(SCALE)% POINTS markers use x and y scale for width/height respectively
			points.scale.y = 2;
			points.color.g = 1.0f;// %Tag(COLOR)% // Points are green
			points.color.a = 1.0;

			geometry_msgs::Point p;
			p.x = x0;
			p.y = y0;
			points.points.push_back(p);

			marker_pub.publish(points);
		}



		//rate.sleep();

/*



		if(i<50){  // need to publish for 10+ times and then received by rviz, why?
			// Publish waypints using visualization_msgs::Marker::POINTS================================
			visualization_msgs::Marker points;// %Tag(MARKER_INIT)%
			points.header.frame_id = "frame";
			points.header.stamp = ros::Time::now();
			points.ns = "waypoints";
			points.action = visualization_msgs::Marker::ADD;
			points.pose.orientation.w = 1.0;
			points.id = 0; // %Tag(ID)%
			points.type = visualization_msgs::Marker::POINTS; // %Tag(TYPE)%
			points.scale.x = 0.2;// %Tag(SCALE)% POINTS markers use x and y scale for width/height respectively
			points.scale.y = 0.2;
			points.color.g = 1.0f;// %Tag(COLOR)% // Points are green
			points.color.a = 1.0;

			for(int j = 0; j < wps_num; j++){
				geometry_msgs::Point p;
				p.x = wps_x[j];
				p.y = wps_y[j];
				points.points.push_back(p);
			}
			marker_pub.publish(points);

			// Publish raw waypints using visualization_msgs::Marker::POINTS=============================
			visualization_msgs::Marker points1;// %Tag(MARKER_INIT)%
			points1.header.frame_id = "frame";
			points1.header.stamp = ros::Time::now();
			points1.ns = "waypoints_raw";
			points1.action = visualization_msgs::Marker::ADD;
			points1.pose.orientation.w = 1.0;
			points1.id = 0; // %Tag(ID)%
			points1.type = visualization_msgs::Marker::POINTS; // %Tag(TYPE)%
			points1.scale.x = 0.5;// %Tag(SCALE)% POINTS markers use x and y scale for width/height respectively
			points1.scale.y = 0.5;
			points1.color.g = 1.0f;// %Tag(COLOR)% // Points are green
			points1.color.r = 0.0f;// red
			points1.color.a = 1.0;

			for(int j = 0; j < wps_num_raw; j++){
				geometry_msgs::Point p1;
				p1.x = wps_x_raw[j];
				p1.y = wps_y_raw[j];
				points1.points.push_back(p1);
			}
			marker_pub.publish(points1);


			// Publish waypoint's label using Marker
			for(int j = 0; j < wps_num; j++){
				visualization_msgs::Marker label;
				label.header.frame_id="frame";
				label.header.stamp = ros::Time::now();
				label.ns = "waypoint_label";
				label.action = visualization_msgs::Marker::ADD;
				label.pose.orientation.w = 1.0;
				label.id =j; // Marker id should be unique. Any marker sent with the same namespace and id will overwrite the old one
				label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

				label.scale.x = 0.5;
				label.scale.y = 0.5;
				label.scale.z = 0.5;

				label.color.b = 1.0f;
				label.color.g = 1.0f;
				label.color.r = 1.0f;
				label.color.a = 1.0;

				label.pose.position.x =  wps_x[j];  //???????????????????????????????????????????
				label.pose.position.y =  wps_y[j];
				label.pose.position.z =  -2.0;
				label.pose.orientation.x = 0.0;
				label.pose.orientation.y = 0.0;
				label.pose.orientation.z = 0.0;
				label.pose.orientation.w = 1.0;
				ostringstream str;
				str<<j;
				label.text=str.str();
				marker_pub.publish(label);
			}

		}


		// publish vehicle's coordinates: Marker - line_list ====================================================
		visualization_msgs::Marker line_list;// %Tag(MARKER_INIT)%
		line_list.header.frame_id = "frame";
		line_list.header.stamp = ros::Time::now();
		line_list.ns = "mkz_line_list";
		line_list.action = visualization_msgs::Marker::ADD;
		line_list.pose.orientation.w = 1.0;
		line_list.id = i; // Marker id should be unique
		line_list.type = visualization_msgs::Marker::LINE_LIST;
		line_list.scale.x = 0.2;// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
		line_list.color.r = 1.0; //red
		line_list.color.a = 1.0;
		geometry_msgs::Point p;
		p.x = x0;
		p.y = y0;
		line_list.points.push_back(p);
		p.x = x0_frt;
		p.y = y0_frt;
		line_list.points.push_back(p);
		marker_pub.publish(line_list);

		// publish 3d vehicle frame: Marker - cube  =================================================================
		visualization_msgs::Marker box1;
		visualization_msgs::Marker box2;
		box1.header.frame_id = box2.header.frame_id = "frame";
		box1.header.stamp = box2.header.stamp = ros::Time::now();
		box1.ns = "3d_vehicle_cubic1";
		box2.ns = "3d_vehicle_cubic2";
		box1.id = 0;
		box2.id = 0;
		uint32_t shape = visualization_msgs::Marker::CUBE;
		box1.type = box2.type = shape;
		box1.action = box2.action = visualization_msgs::Marker::ADD;
		box1.pose.position.x = x0;
		box1.pose.position.y = y0;
		box1.pose.position.z = 2;
		box2.pose.position.x = x0_frt;
		box2.pose.position.y = y0_frt;
		box2.pose.position.z = 1.6;
		//box.pose.orientation.x = 1.0;
		//box.pose.orientation.y = 1.0;
		//box.pose.orientation.z = 1.0;
		//box.pose.orientation.w = 1.0;
		box1.pose.orientation = box2.pose.orientation = tf::createQuaternionMsgFromYaw(hedg); //convert euler (yaw) to quaternion for rviz visualization; header file: <tf/transform_datatypes.h>

		box1.scale.x = 4.0;
		box1.scale.y = 2.3;
		box1.scale.z = 2.5;

		box2.scale.x = 1.2;
		box2.scale.y = 2.0;
		box2.scale.z = 1.8;

		box1.color.r = box2.color.r = 1.0f;
		box1.color.g = box2.color.g = 1.0f;
		box1.color.b = box2.color.b = 1.0f;
		box1.color.a = 0.8;
		box2.color.a = 1.0;

		box1.lifetime = box2.lifetime = ros::Duration();
		marker_pub.publish(box1);
		marker_pub.publish(box2);


		//publish 4 wheels: Marker - cylinder ======================================================================

		float frx, fry, flx, fly, rrx, rry, rlx, rly;
		float d1 = 2.5; // wheelbase
		float d2 = 1.0; // half track distance

		frx = x0 + cos(hedg)*d1 - sin(hedg)*(-d2);
		fry = y0 + sin(hedg)*d1 + cos(hedg)*(-d2);
		flx = x0 + cos(hedg)*d1 - sin(hedg)*(d2);
		fly = y0 + sin(hedg)*d1 + cos(hedg)*(d2);
		rrx = x0 - sin(hedg)*(-d2);
		rry = y0 + cos(hedg)*(-d2);
		rlx = x0 - sin(hedg)*(d2);
		rly = y0 + cos(hedg)*(d2);

		visualization_msgs::Marker frw, flw, rrw, rlw; //front left wheel
		frw.header.frame_id = flw.header.frame_id = rrw.header.frame_id = rlw.header.frame_id = "frame";
		frw.header.stamp = flw.header.stamp = rrw.header.stamp = rlw.header.stamp = ros::Time::now();
		frw.ns = flw.ns = rrw.ns = rlw.ns = "wheel";
		frw.id = 0;
		flw.id = 1;
		rrw.id = 2;
		rlw.id = 3;

		uint32_t shape1 = visualization_msgs::Marker::CYLINDER;

		frw.type = flw.type = rrw.type = rlw.type = shape1;
		frw.action = flw.action = rlw.action = rrw.action = visualization_msgs::Marker::ADD;
		frw.pose.position.x = frx;
		frw.pose.position.y = fry;
		flw.pose.position.x = flx;
		flw.pose.position.y = fly;
		rrw.pose.position.x = rrx;
		rrw.pose.position.y = rry;
		rlw.pose.position.x = rlx;
		rlw.pose.position.y = rly;

		frw.pose.orientation = flw.pose.orientation = rrw.pose.orientation = rlw.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( 0.0f, 1.5707f,hedg+1.5707f); //convert euler (yaw) to quaternion for rviz visualization; header file: <tf/transform_datatypes.h>

		frw.scale.x = flw.scale.x = rrw.scale.x = rlw.scale.x = 1.0;
		frw.scale.y = flw.scale.y = rrw.scale.y = rlw.scale.y = 1.0;
		frw.scale.z = flw.scale.z = 0.25;
		rrw.scale.z = rlw.scale.z = 0.5;

		frw.color.r = flw.color.r = rrw.color.r = rlw.color.r = 0.0f;
		frw.color.g = flw.color.g = rrw.color.g = rlw.color.g = 0.0f;
		frw.color.b = flw.color.b = rrw.color.b = rlw.color.b = 0.0f;
		frw.color.a = flw.color.a = rrw.color.a = rlw.color.a = 1.0;

		frw.lifetime = flw.lifetime = rrw.lifetime = rlw.lifetime = ros::Duration();

		marker_pub.publish(frw);
		marker_pub.publish(flw);
		marker_pub.publish(rrw);
		marker_pub.publish(rlw);


		//publish ISUZU logos using visualization_msgs::Marker::TEXT_VIEW_FACING  ======================================================================

		float logo1x, logo1y;
		d1 = 0.0; // wheelbase
		d2 = 0.0; // half track distance

		logo1x = x0 + cos(hedg)*d1 - sin(hedg)*(-d2);
		logo1y = y0 + sin(hedg)*d1 + cos(hedg)*(-d2);


		visualization_msgs::Marker logo1;
		logo1.header.frame_id = "frame";
		logo1.header.stamp = ros::Time::now();
		logo1.ns = "logo";
		logo1.id = 0;


		uint32_t shape2 = visualization_msgs::Marker::TEXT_VIEW_FACING;

		logo1.type = shape2;
		logo1.action = visualization_msgs::Marker::ADD;
		logo1.pose.position.x = logo1x;
		logo1.pose.position.y = logo1y;
		logo1.pose.position.z = 4;


		logo1.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( 0.0f, 0.0f,hedg+1.5707f);

		logo1.scale.z = 0.8;

		logo1.color.r = 1.0f;
		logo1.color.g = 0.0f;
		logo1.color.b = 0.0f;
		logo1.color.a = 1.0;

		logo1.lifetime = ros::Duration();

		logo1.text="ISUZU";

		marker_pub.publish(logo1);



		// broadcast another frame:  parent: "frame" -> child "car" (to add a target view on the vehicle)=======================================
		transform.setOrigin( tf::Vector3(x0, y0, 0.0) );
		transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "frame", "car"));


*/

	}
	return 0;


}
