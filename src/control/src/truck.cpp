//0918 mx 0924 pz 0928 mx 1116 mx

#include <ros/ros.h>
#include <ros/package.h> // for ros::package::getPath()

#include "std_msgs/String.h" // for sending msg type: std_msgs/String
#include "std_msgs/Int16.h" // for sending msg type: std_msgs/String
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h" // for lat lon, 32bit not accurate
#include "std_msgs/Int16MultiArray.h"

#include <iostream>
#include <fstream>
#include <iomanip>  // for cout double with precision(6)
#include <cmath>

#include <string>  // for udp socket programming - client
#include <sys/socket.h> // for udp socket programming - client
#include <arpa/inet.h> // for udp socket programming - client
#include <unistd.h> // for udp socket programming - client
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <netinet/in.h>

//#define PORT 8080  // for udp socket programming - client
//#define PORT 80  // for udp socket programming - client
//#define DEST_PORT 80
//#define DSET_IP_ADDRESS  "192.168.0.2"

#include "boost/asio.hpp"
#include <boost/array.hpp>
#include <boost/bind.hpp>

#include <thread>

#include <math.h>
#include <chrono>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"

using boost::asio::ip::udp;
using namespace std;
#define IPADDRESS "192.168.0.2" // Mbed
#define UDP_PORT 80



#include "std_msgs/Float64.h" //ROS

#include <visualization_msgs/Marker.h>//for publishing markers to rviz

#include <tf/transform_broadcaster.h> // for broadcasting a frame

#include <tf/transform_datatypes.h> // for publishing vehicle yaw using quaternion

#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseStamped.h"
#include "hellocm_msgs/CM2Ext_vhcl.h"
#include "gps_common_msgs/GPSFix.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"

using namespace Eigen;
// MPC is initialized here!
MPC mpc;


int simulation_mode = 0; //-------------------------------------------------->  choose mode here! 0:experiments; 1:simulation - 0918 mx

// std::vector<double> wps_x, wps_y;
double posx,posy,path_yaw;
bool path_loca = false;
bool path_way = false;
int wps_num = 0;
float wps_x[3000], wps_y[3000]; // after removing sharp corners

int run_mode = 2;
// 0 for sensor; 1 for IPG; 2 for Path planning

bool first = true;
float hed_first;

double gnss_arr[2];
float hedg_arr[1];

bool pos_up = false;
bool ang_up = false;

float get_ang (float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4) // angle between 2 vectors
{
	float mag1 = sqrt ((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
	float mag2 = sqrt ((x4-x3)*(x4-x3)+(y4-y3)*(y4-y3));
	float dot_product = (x2-x1)*(x4-x3)+(y2-y1)*(y4-y3);
	float angle = acos(dot_product/mag1/mag2); // in radians
	return angle;
}


float det( float matrix[3][3], int n) {

   float d = 0;
   float submatrix[3][3];
   if (n == 2)
      return ((matrix[0][0] * matrix[1][1]) - (matrix[1][0] * matrix[0][1]));
   else {
      for (int x = 0; x < n; x++) {
            int subi = 0;
            for (int i = 1; i < n; i++) {
               int subj = 0;
               for (int j = 0; j < n; j++) {
                  if (j == x)
                  continue;
                  submatrix[subi][subj] = matrix[i][j];
                  subj++;
               }
               subi++;
            }
            d = d + (pow(-1, x) * matrix[0][x] * det( submatrix, n - 1 ));
      }
   }
   return d;
}


void get_r_center (float x1, float y1, float x2, float y2, float x3, float y3, float &add_r, float &add_cx0, float &add_cy0) // r/curvature/center
{

	x1 = x1*1000;
	y1 = y1*1000;
	x2 = x2*1000;
	y2 = y2*1000;
	x3 = x3*1000;
	y3 = y3*1000;


	float M1[3][3] = { {x1*x1+y1*y1,y1,1.0f}, {x2*x2+y2*y2,y2,1.0f}, {x3*x3+y3*y3,y3,1.0f} };
	float M2[3][3] = { {x1,y1,1.0f},{x2,y2,1.0f},{x3,y3,1.0f} };
	float M3[3][3] = { {x1,x1*x1+y1*y1,1.0f},{x2,x2*x2+y2*y2,1.0f},{x3,x3*x3+y3*y3,1.0f} };

	// argument center cx0 xy0 are passed by giving addresses of memory locations
	add_cx0 = det(M1,3.0f)/det(M2,3.0f)*0.5f/1000;
	add_cy0 = det(M3,3.0f)/det(M2,3.0f)*0.5f/1000;

	//cout << "det1:" << det(M1,3) <<"\n"<< "det2:" << det(M2,3)<<"\n" << "det3:" << det(M3,3) <<"\n";
	// get area
	float vx1 = x2-x1;
	float vy1 = y2-y1;
	float vx2 = x3-x2;
	float vy2 = y3-y2;
	float vx3 = x1-x3;
	float vy3 = y1-y3;
	float A = (vx1*vy2 - vy1*vx2)/2.0f;
	// get length
	float L1 = sqrt( vx1*vx1 + vy1*vy1 );
	float L2 = sqrt( vx2*vx2 + vy2*vy2 );
	float L3 = sqrt( vx3*vx3 + vy3*vy3 );
	// get r
	add_r = L1*L2*L3/4.0f/A/1000;
	if(A == 0){ add_r = 100000;}
}


float VL22_model_r2str (float r)
{
	float str_ang; // tire angle
	float str_wheel_ang; // steering wheel angle position
	float str_ratio = 40.0/900.0;  // dont forget using float data type
	float Lf = 2.76; //in meter
	if (r >= 0){
		str_ang = - atan( Lf / r ); // when r > 0, left turn, str_ang assumed to be < 0
		str_wheel_ang = str_ang / str_ratio;
		str_wheel_ang = str_wheel_ang * 180.0 / 3.1415926;
	}else{

		str_ang = - atan( Lf / r ); // when r < 0, right turn, str_ang assumed to be > 0
		str_wheel_ang = str_ang / str_ratio;
		str_wheel_ang = str_wheel_ang * 180.0 / 3.1415926;
	}
	if (str_wheel_ang > 800){
		str_wheel_ang = 800;
		cout << "curvature too high!" << "\n";
	}
	if (str_wheel_ang < -800){
		str_wheel_ang = -800;
		cout << "curvature too high!" << "\n";
	}
	return str_wheel_ang;
}

float MKZ_model_r2str (float r)
{
	int str_step;
	if (r >= 0){
		str_step = atan(-4.8f/(r-0.802464f)) * 5000.0f ; // left turn with r > 0, str_step < 0
	}else{
		str_step = atan(-4.8f/(r+0.802464f)) * 5000.0f ; // right turn with r < 0, str_step >0
	}
	if (str_step > 5000){
		str_step = 5000;
		cout << "curvature too high!" << "\n";
	}
	if (str_step < -5000){
		str_step = -5000;
		cout << "curvature too high!" << "\n";
	}
	return str_step;
}

int find_side_2_line (float x0, float y0, float x1, float y1, float x2, float y2)
{
	int side;
	float A = y2 - y1;
	float B = x1 - x2;
	float C = x2*y1 - x1*y2;
	float D = A*x0 + B*y0 +C;
	if ( D > 0 ){
		side = -1; // on the left
	}else if ( D < 0 ){
		side = 1; // on the right
	}else{
		side = 0; // on the line
	}
	return side;
}

float find_d_point_2_line ( float x0, float y0, float x1, float y1, float x2, float y2 ) // distance btw point to line defined by 2 points
{
	float A = y2 -y1;
	float B = x1 - x2;
	float C = x2*y1 - x1*y2;
	float d = abs( A*x0 + B*y0 + C) / sqrt( A*A + B*B );
	return d;
}


float hedg2ENU (float hedg) // for simulation, range from -Pi to Pi
{
	hedg = 3.1415926f*2.0f - hedg/180.0f*3.1415926f;
	hedg = hedg - 3.1415926f*3.0f/2.0f;
	if ( hedg < -3.1415926f ){
		hedg = hedg + 2.0f*3.1415926f;
	}
	return hedg;
}

void MKZ_model ( float x0, float y0, float hedg, float str_step, float d_travel, float &add_x0, float &add_y0, float &add_hedg )//for simulation, hedg in radian
{
	if (str_step > 5000){
		str_step = 5000;
	}
	if (str_step < -5000){
		str_step = -5000;
	}

	str_step = -str_step; // due to an issue in matlab code
	float d1 = 0.88; // width/2
	float d2 = 1.082; // rear axial to rear end
	float d3 = 3.843; // rear axial to front end
	float dhedg;
	float r;
	float str_step_max = 5000.0;
	if (str_step == 0){
		add_x0 = x0 + cos(hedg)*d_travel;
		add_y0 = y0 + sin(hedg)*d_travel;
		add_hedg =	hedg;
	}else{
		if (str_step >= 0){
			r = -4.8f/tan( str_step/str_step_max ) - 0.802464f;
		}else{
			r = -4.8f/tan( str_step/str_step_max ) + 0.802464f;
		}
		dhedg = - d_travel/r;
		float dd1 = sin(dhedg)*r;
		float dd2 = r - cos(dhedg)*r;
		add_x0 = x0 + cos(3.1415926f + hedg)*dd1 - sin(3.1415926f + hedg)*dd2;
		add_y0 = y0 + sin(3.1415926f + hedg)*dd1 + cos(3.1415926f + hedg)*dd2;
		add_hedg = hedg + dhedg;
		if( add_hedg > 3.1415926f ){
			add_hedg = add_hedg - 2.0f*3.1415926f;
		}else if( add_hedg < - 3.1415926f ){
			add_hedg = add_hedg + 2.0f*3.1415926f;
		}

	}
}

void dynamic_model ( float x0, float y0, float hedg, float str_step, float d_travel, float &add_x0, float &add_y0, float &add_hedg )//for simulation, hedg in radian
{
	float theta; //str angle
	float Vx = 0.1; // m/s vehicle speed
	float dt = 0.05; // seconds
	float lf = 3; //meter
	float lr = 1; //meter
	float Caf = 1; // conering stiffness??
	float Car = 1;
	float m = 1000; // Kg vechile weight
	float Iz = 500; // moment of inertia for z axis
	float ax = 0; //accleration in local x, assumming zero;
	float y,  y_new,  x,  x_new,  yaw,  yaw_new, X, X_new, Y, Y_new;
	float dy, dy_new, dx, dx_new, dyaw, dyaw_new;

	y = 0; // local coordinate
	x = 0;
	yaw = hedg;
	X = x0; //global coordinate
	Y = y0;

	dy = 0;
	dx = Vx;
	dyaw = 0;


	if (str_step > 5000){
		str_step = 5000;
	}
	if (str_step < -5000){
		str_step = -5000;
	}
	str_step = -str_step; // due to an issue in matlab code
	theta = str_step/5000.0f*540.0f*3.14f/180.0f; // str_angle

	for(int i = 0; i < 20; i++)
	{
		y_new = y + dy*dt;
		dy_new = dy - 2*(Caf*lf - Car*lr)*dy*dt/m/Vx - Vx*dyaw*dt - 2*(Caf*lf-Car*lr)*dyaw*dt/m/Vx + 2*Caf*theta*dt/m;
		yaw_new = yaw + dyaw*dt;
		dyaw_new = dyaw - 2*(Caf*lf-Car*lr)*dy*dt/Iz/Vx - 2*(Caf*lf*lf+Car*lr*lr)*dyaw*dt/Iz/Vx + 2*lf*Caf*theta*dt/Iz;
		dx_new = dx + dyaw_new*dy_new*dt + ax*dt;
		x_new = x + dx_new*dt;

		X_new = X + (dx*cos(yaw) - dy*sin(yaw))*dt;
		Y_new = Y + (dx*sin(yaw) + dy*cos(yaw))*dt;

		y = y_new;
		dy = dy_new;
		yaw = yaw_new;
		dyaw = dyaw_new;
		x = x_new;
		dx = dx_new;

		X = X_new;
		Y = Y_new;
	}

	add_x0 =  X_new;
	add_y0 =  Y_new;
	add_hedg = yaw_new;
}


void gnssCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{

	int i = 0;


	for(std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it) // "double" consistent to "Float64" 
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

void gnssIPGCallback(const gps_common_msgs::GPSFix::ConstPtr &msg)
{

	gnss_arr[0] = msg->latitude;
	gnss_arr[1] = msg->longitude;

	int j = 0;
	for(j = 0; j < 2; j++)
	{
	printf("%0.7f, ", gnss_arr[j]);
	}
	printf("\n");

	// ROS_INFO("I heard gnss: [%0.7f, %0.7f]", gnss_arr[0],gnss_arr[1]);

	pos_up = true;

	return;
}
void hedgIPGCallback(const hellocm_msgs::CM2Ext_vhcl::ConstPtr &msg)
{
	hedg_arr[0] = msg->heading; // 0 deg point East, anticlockwise
	hedg_arr[1] = msg->pitch;
	hedg_arr[2] = msg->roll;	

	int j = 0;
	for(j = 0; j < 1; j++)
	{
	printf("%f, ", hedg_arr[j]);
	}
	printf("\n");

	ROS_INFO("I heard heading: [%f]", hedg_arr[0]);

	ang_up = true;

	if (first == true)
	{
		hed_first = hedg_arr[0];
		first = false;
	}	

	return;
}

/*!
 * Description:
 * - Callback for ROS Topic published by path planning
 */
void locateCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    ROS_INFO("updating localization");
	path_loca = true;
	// double posx,posy,path_yaw;
	posx = msg->pose.pose.position.x;
	posy = msg->pose.pose.position.y;

	tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
	path_yaw = yaw;//+hedg2ENU(hed_first)+M_PI/2;

	ROS_INFO("yaw: %0.9f;",yaw);
	ROS_INFO("posx: %0.9f; posy: %0.9f; yaw: %0.9f",posx, posy, path_yaw);
    // ROS_INFO("Yaw: %0.9f", rad2deg(path_yaw));
}
void pathCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
	if (msg->poses.size()>0)
	{
        
		ROS_INFO("receiving waypoints");
        // wps_x.clear();
        // wps_y.clear();
		
		int wps_cnt = 0;
		for (size_t i = 0; i < msg->poses.size(); i++)
		{
			// wps_x.push_back(msg->poses[i].position.x);
			// wps_y.push_back(msg->poses[i].position.y);
			if ( i%2 ==1)
			{
				wps_x[wps_cnt] = msg->poses[i].position.x;
				wps_y[wps_cnt] = msg->poses[i].position.y;
				wps_cnt++;
			}
			
			// wps_x[i] = msg->poses[i].position.x;
			// wps_y[i] = msg->poses[i].position.y;
		}
        // wps_num = msg->poses.size();
		wps_num = wps_cnt;
		ROS_INFO("wps_num: %i", wps_num);
		for (size_t i_c = 0; i_c < wps_num; i_c++)
		{
			cout << "wpx_" << i_c << ":" << wps_x[i_c] << "\n";
			cout << "wpy_" << i_c << ":" << wps_y[i_c] << "\n";
		}
		
		ROS_INFO("Size: %i, [1]= %lf, [2]= %lf,[3]= %lf,[4]= %lf,[5]= %lf",msg->poses.size(),wps_x[0],wps_x[1],wps_x[2],wps_x[3],wps_x[4]);
		path_way = true;
	}		
}

void GNSS_read()
{
	while(ros::ok())
	{
		ros::spinOnce();
	}
}


// MPC - Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

void mpc_fun(vector<double> ptsx, vector<double> ptsy, double px, double py, double psi, double v, double &steer_value, double &throttle_value, vector<double> &mpc_x_vals, vector<double> &mpc_y_vals, vector<double> &next_x_vals, vector<double> &next_y_vals, vector<double> &mpc_x_ref, vector<double> &mpc_y_ref) {
    /*
    * Calculate steering angle and throttle using MPC.
    *
    * Both are in between [-1, 1].
    *
    */
    VectorXd waypoint_x_vcs = VectorXd(ptsx.size());
    VectorXd waypoint_y_vcs = VectorXd(ptsy.size());

    // transfrom from map coordinate system to vehicle coordinate system (VCS)
    for (size_t i = 0; i < ptsx.size(); ++i) {
        double dx = ptsx[i] - px;
        double dy = ptsy[i] - py;
        waypoint_x_vcs[i] = dx * cos(psi) + dy * sin(psi);
        waypoint_y_vcs[i] = -dx * sin(psi) + dy * cos(psi);
    }

    auto coeffs = polyfit(waypoint_x_vcs, waypoint_y_vcs, 3); //  3rd order polynomial fitting
    // since we use the vehicle coordinates, the x is 0
    auto cte = polyeval(coeffs, 0);
    auto epsi = -atan(coeffs[1]);

    Eigen::VectorXd state(6);
    state << 0, 0, 0, v, cte, epsi;

    std::vector<double> x_vals = {state[0]};
    std::vector<double> y_vals = {state[1]};
    std::vector<double> psi_vals = {state[2]};
    std::vector<double> v_vals = {state[3]};
    std::vector<double> cte_vals = {state[4]};
    std::vector<double> epsi_vals = {state[5]};
    std::vector<double> delta_vals = {};
    std::vector<double> a_vals = {};

    // compute the optimal trajectory
    auto result = mpc.Solve(state, coeffs);

    // update the previous steering and acceleration
    mpc.setPrevDelta(result[0]);
    mpc.setPrevA(result[1]);

    steer_value = -result[0];
    throttle_value = result[1];

    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
    // msgJson["steering_angle"] = steer_value / deg2rad(25);
    // msgJson["throttle"] = throttle_value;

    //Display the MPC predicted trajectory
    mpc_x_vals = mpc.getMpcX();
    mpc_y_vals = mpc.getMpcY();

    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
    // the points in the simulator are connected by a Green line
    // msgJson["mpc_x"] = mpc_x_vals;
    // msgJson["mpc_y"] = mpc_y_vals;

    //Display the waypoints/reference line
    //vector<double> next_x_vals(ptsx.size()); 
    //vector<double> next_y_vals(ptsx.size());

    for (size_t i = 0; i < ptsx.size(); ++i) {
        next_x_vals[i] = waypoint_x_vcs[i];
        next_y_vals[i] = waypoint_y_vcs[i];
    }
								


    unsigned long long num_ptsx = ptsx.size();
    double dptsx = ( next_x_vals[num_ptsx-1] - next_x_vals[0] ) /10;	 
    //cout << "dptsx =" << dptsx << endl; 


    for (size_t i = 0; i < 11; ++i) {  
	mpc_x_ref[i] = next_x_vals[0] + i*dptsx;
	mpc_y_ref[i] = polyeval(coeffs, mpc_x_ref[i]);
    }



    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
    // the points in the simulator are connected by a Yellow line
    // msgJson["next_x"] = next_x_vals;
    // msgJson["next_y"] = next_y_vals;
}


float trj_plan_pp ( float x0, float y0, float hedg, float x1, float y1, float x2, float y2 ) // pp - pure pursuit
{
	float str_ang_max = 540; //                                           ------------> double check truck's parameters
	float str_step_arr[23] = {-22500, -20000, -17500, -15000, -12500, -10000, -7500, -5000, -2500, -1000, -500, 0, 500, 1000, 2500, 5000, 7500, 10000, 12500, 15000, 17500, 20000, 22500};
	float str_step, str_step_opt;	
	float str_step_max = 25000;
	float str_ang, str_ang_opt;	
	float d_travel = 3.0; //meter
	float str_ang_cmd;
	float d1 = 0.84; // width/2 for mkz                                    -----------> double check turck's parameters
	float d2 = 1.082; // rear axial to rear end for mkz                    -----------> double check turck's parameters
	float d3 = 3.843; // rear axial to front end for mkz                   -----------> double check turck's parameters
	float dhedg;
	float r;
	float x0_out;
	float y0_out;
	float hedg_out;
	float x0_frt, y0_frt;// vehicle front head 
	float d_frt = 3; // vehicle front head distance (m)
	float ang1, ang2;
	float cost;
	float cost1;
	float cost2;	
	float cost_min = 10000;
	float dd1, dd2;	

	for (int i = 0; i < 23; i++){
		str_step = str_step_arr[i];

		str_step = -str_step; // due to an issue in matlab code

		// truck kinematic model
		if (str_step == 0){
			x0_out = x0 + cos(hedg)*d_travel; 
			y0_out = y0 + sin(hedg)*d_travel;
			hedg_out = hedg;
		}else{
			r = 4.8/tan( abs(str_step)/str_step_max ) - 1.9; // for truck
			if (str_step >= 0){
				r = -r;
			}
			dhedg = -d_travel/r; 
			float dd1 = sin(dhedg)*r;
			float dd2 = r - cos(dhedg)*r;
			x0_out = x0 + cos(3.1415926 + hedg)*dd1 - sin(3.1415926 + hedg)*dd2;
			y0_out = y0 + sin(3.1415926 + hedg)*dd1 + cos(3.1415926 + hedg)*dd2;
			hedg_out = hedg + dhedg;
		}


		// evaluate cost functions
		
		x0_frt = x0_out + d_frt*cos(hedg); 
		y0_frt = y0_out + d_frt*sin(hedg);	
		ang1 = get_ang(x0_out, y0_out, x2, y2, x0_out, y0_out, x0_frt, y0_frt); //vehicle to p2 vector and segment vector angle
		ang2 = get_ang(x0_out, y0_out, x0_frt, y0_frt, x1, y1, x2, y2); // vehicle heading vector and segment vector angle
		cost1 = ang1;
		cost2 = ang2;
		cost = 0.9*ang1 + 0.1*ang2;
		if ( cost < cost_min ){
			str_step_opt = str_step_arr[i];
			//str_ang_opt = str_step_opt/str_step_max*810; // relation ????????????????????????????????????????????????????????????????????????
			cost_min = cost;
		}
		cout <<"0928 debug ---->   str_angle =" << str_step;
		cout <<";   cost =" << cost << "\n";
		cout << "x0, y0, x1,y1,x2,y2: " <<  x0  << y0<<x1<<y1<<x2<<y2<<"\n";


	}


	//cout << "optimized steering angle = " << str_ang << "\n";
	return str_step_opt;
}


void truck_kinematics_model ( float x0, float y0, float hedg, float str_step, float d_travel, float &add_x0, float &add_y0, float &add_hedg )//for simulation, hedg in radian
{
	if (str_step > 25000){
		str_step = 25000;
	}
	if (str_step < -25000){
		str_step = -25000;
	}

	str_step = -str_step; // due to an issue in matlab code
	float d1 = 0.88; // width/2
	float d2 = 1.082; // rear axial to rear end
	float d3 = 3.843; // rear axial to front end
	float dhedg;
	float r;
	float str_step_max = 25000.0;


	// truck kinematic model
	if (str_step == 0){
		add_x0 = x0 + cos(hedg)*d_travel; 
		add_y0 = y0 + sin(hedg)*d_travel;
		add_hedg = hedg;
	}else{
		r = 4.8/tan( abs(str_step)/str_step_max ) - 1.9; // for truck
		if (str_step >= 0){
			r = -r;
		}
		dhedg = -d_travel/r; 
		float dd1 = sin(dhedg)*r;
		float dd2 = r - cos(dhedg)*r;
		add_x0 = x0 + cos(3.1415926 + hedg)*dd1 - sin(3.1415926 + hedg)*dd2;
		add_y0 = y0 + sin(3.1415926 + hedg)*dd1 + cos(3.1415926 + hedg)*dd2;
		add_hedg = hedg + dhedg;
	}

}




int main ( int argc, char**argv){

	//Initialize the ROS system.
	ros::init(argc, argv, "mkz");

	// Establish this program as a ROS node.
	ros::NodeHandle n;

	// Create publisher object
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_markers", 1000);

	ros::Publisher float_pub = n.advertise<std_msgs::Float64>("float", 1000);


	ros::Publisher str_pub = n.advertise<std_msgs::Int16MultiArray>("str_wheel_ang_cmd", 1000);
	ros::Publisher str_IPG_pub = n.advertise<std_msgs::Int16>("steering_angle", 1000);





	ros::Subscriber gnss_sub = n.subscribe("gnss_msg", 1, gnssCallback);
	ros::Subscriber hedg_sub = n.subscribe("ang_msg", 1, hedgCallback);

	ros::Subscriber loca_path_sub = n.subscribe("gps_ref", 1, locateCallback);
	ros::Subscriber upda_path_sub = n.subscribe("waypoints_ref", 1, pathCallback);
		
	ros::spinOnce();
	thread GNSSread(GNSS_read);

  	ros::Rate rate(30);

	//Send some output as a log message.
	ROS_INFO_STREAM("Hello, ROS!");

	// Broadcast a frame
	tf::TransformBroadcaster br;
	tf::Transform transform;


	//Parameters

	int i = 1;

	double lon, lat;
	float x0, y0, hedg; // vehicle ENU coordinate
	float x0_frt, y0_frt;// vehicle front head
	float d_frt = 3.0 ; // vehicle front head distance (m)
	// float d_frt = 5.0 ;
	double wps_lon[3000], wps_lat[3000];
	float wps_x_raw[3000], wps_y_raw[3000];
	// float wps_x[3000], wps_y[3000]; // after removing sharp corners
	double in_lat, in_lon;
	int lat_num;
	int lon_num;
	int wps_num_raw;
	// int wps_num = 0;
	int wps_num_0 = 10000;
	double lat_ref, lon_ref;
	int idx_glb, idx;
	//float x1, x2, y1, y2, x3, y3;
	double x1, x2, y1, y2, x3, y3, x4, y4, x5, y5, x6, y6;
	float ang1, ang2, ang3;
	float d_v2seg_min, d_v2wps_min;
	float d_v2seg, d_v2wps;
	int wps_idx; //global 0, 1, 2, ...
	int wps_wdw_num = 14; //  window ahead with wps_idx
	int wps_wdw[wps_wdw_num];
	int wps_wdw_idx;
	float r, cx0, cy0;
	int str_ctrl_mode;
	float d_o2v; // circle center to vechile
	int Kp, Kd;
	float offset_ctrl;
	float offset_ctrl_pre = 0;
	float d_offset;
	float str_step_ref;
	float str_wheel_ang_ref;
	float str_step;
	float str_wheel_ang;
	int str_cmd; // must be int
	float d_p2line;
	int side2line;

	float theta1, theta3, d_theta;// for plotting trajectory plan
	float traj_x[100], traj_y[100];


	float d_travel = 0.1;//simulation

	//double add_x0, add_y0, add_hedg; // simulation
	float add_x0, add_y0, add_hedg; // simulation
	float add_cx0, add_cy0, add_r;


	vector<double> mpc_x_vals;
	vector<double> mpc_y_vals;

	vector<double> next_x_vals;
	vector<double> next_y_vals;

	vector<double> mpc_x_ref; // reference trajectory by polyfit
	vector<double> mpc_y_ref;


	// print to user
	cout << "Hello World!" << endl;


	// Load map

	std::string pkg_loc = ros::package::getPath("dbw");	// add header file <ros/package.h>; add std_msgs roslib to cmake and xml files--------------------??????????????????????????
	cout << "The path is :" << pkg_loc << endl;

	if (run_mode != 2 )
	{
		ifstream inFile_lat(pkg_loc+ "/lat_raw.txt");
		if (inFile_lat.is_open())
		{
			lat_num = 0;
			while ( inFile_lat >> in_lat )
			{

				//cout << fixed << setprecision(8) << in_lat << '\n';
				wps_lat[lat_num] = in_lat;
				//cout << lat_raw[lat_num] << '\n';
				lat_num = lat_num + 1;
			}
			inFile_lat.close();
			cout << "lat_num is equal to " << lat_num << '\n';
		}else{
			cout << "Unable to open file lat_raw.txt";
			exit(1); // call system to stop
		}


		ifstream inFile_lon(pkg_loc + "/lon_raw.txt");
		if (inFile_lon.is_open())
		{
			lon_num = 0;
			while ( inFile_lon >> in_lon )
			{

				//cout << fixed << setprecision(8) << in_lon << '\n';
				wps_lon[lon_num] = in_lon;
				//cout << lon_raw[lon_num] << '\n';
				lon_num = lon_num + 1;
			}
			inFile_lon.close();
			cout << "lon_num is equal to " << lon_num << '\n';
		}else{
			cout << "Unable to open file lon_raw.txt";
			exit(1); // call system to stop
		}
		if (lat_num == lon_num)
		{
			wps_num_raw = lat_num;
		}else{
			cout << "lon_num not equal to lat_num, check lon_raw.txt lat_raw.txt";
			exit(1); // call system to stop
		}
		cout << "wps_num_raw =" << wps_num_raw << "\n";

		// Geodetic to ENU
		lon_ref = wps_lon[0];
		lat_ref = wps_lat[0];
		for (int j = 0; j < wps_num_raw; j++) {
			wps_x_raw[j] = (wps_lon[j] - lon_ref)*82230.670304;
			wps_y_raw[j] = (wps_lat[j] - lat_ref)*111132.944444;
			//cout << "wps_x" << j << ":" << wps_x[j] << "\n";
			//cout << "wps_y" << j << ":" << wps_y[j] << "\n";
		}

		
			// debug =======================
		for(int j=0; j<wps_num_raw; j++){
			wps_x[j] = wps_x_raw[j];
			wps_y[j] = wps_y_raw[j];
		}
		wps_num = wps_num_raw;
		


		// Remove sharp corners from the raw waypoints===========================================
		wps_x[0] = wps_x_raw[0];
		wps_y[0] = wps_y_raw[0];

		/*
		while( wps_num != wps_num_0 ){
			wps_num_0 = wps_num;
			int k = 0;
			for (int j = 0; j < wps_num_raw-2; j++){
				x1 = wps_x_raw[j];
				y1 = wps_y_raw[j];
				x2 = wps_x_raw[j+1];
				y2 = wps_y_raw[j+1];
				x3 = wps_x_raw[j+2];
				y3 = wps_y_raw[j+2];
				get_r_center (x1, y1, x2, y2, x3, y3, r, cx0, cy0);

				if(r < -6 || r > 6 ){
					k = k + 1;
					wps_x[k] = x2;
					wps_y[k] = y2;
				}
			}
			wps_num = k+2;
			wps_x[wps_num-1] = wps_x_raw[wps_num_raw]; // keep the last waypoint the same
			wps_y[wps_num-1] = wps_y_raw[wps_num_raw];
		}
		*/
		cout << "wps_num =" << wps_num << "\n";



		// Get GPS  =================================================================================

		//uint8_t init_msg[5] = {103,110, 115, 115, 49}; // ascii "gnss1"
		//uint8_t *hello = init_msg;
		//uint8_t buffer[10] = {0};

		/*
		int sock = 0, valread;
		struct sockaddr_in serv_addr;

		if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
		{
			printf("\n Socket creation error \n");
			return -1;
		}
		serv_addr.sin_family = AF_INET;
		serv_addr.sin_port = htons(PORT);
		//if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0) // Convert IPv4 and IPv6 addresses from text to binary form
		if(inet_pton(AF_INET, "192.168.0.2", &serv_addr.sin_addr)<=0) // Convert IPv4 and IPv6 addresses from text to binary form
		{
			printf("\nInvalid address/ Address not supported \n");
			return -1;
		}


		if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)   // ?????????????????????????????????????????????????????????????????????????
		{
			printf("\nConnection Failed!! \n");
			return -1;
		}

		send(sock , hello , 10, 0 ); //send(sock , hello , strlen(hello) , 0 );
		//printf("Hello message sent\n");

		valread = read( sock , buffer, 10);

		//printf("%s\n",buffer );
		//printf("%d\n",buffer[0] ); 	// if printing only one element, use %d
		*/


		//---------------------------------------------------------------------------------------------------------------

		/*weiyang version*/

		/*

		uint8_t init_msg[5] = {115,109, 100, 0, 255};  // initialization steering control
		//uint8_t call_gnss[5] = {103,110, 115, 115, 49}; // ascii "gnss1"

		uint8_t steer_cg_msg[5] = {115, 98, 119, 168, 97};


		uint8_t reset_msg[5] = {115, 98, 119, 0, 0}; // steering angle go to zero
		char recv_buf[25];

		boost::asio::io_service io_service;

		udp::socket socket(io_service);

		udp::endpoint receiver_endpoint(boost::asio::ip::address::from_string(IPADDRESS), UDP_PORT); //Mbed

		socket.open(udp::v4());


		boost::system::error_code ignored_error;
		//boost::system::error_code error;

		socket.send_to(boost::asio::buffer(init_msg), receiver_endpoint, 0, ignored_error);
		cout << "steering initilized!" << endl;

		sleep(2);

		socket.send_to(boost::asio::buffer(steer_cg_msg), receiver_endpoint, 0, ignored_error);
		cout << "send steering command!" << endl;

		sleep(2);

		//socket.send_to(boost::asio::buffer(reset_msg), receiver_endpoint, 0, ignored_error);
		//cout << "send steering command!" << endl;

		cout<< " test!!!!!!!!!!!!!!!!  "<<endl;


		socket.bind(udp::endpoint(boost::asio::ip::address::from_string(IPADDRESS), UDP_PORT));	//must bind if receive data over udp


		// size_t len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
		socket.receive_from(boost::asio::buffer(recv_buf), receiver_endpoint);
		// cout << "len: " << len << endl;

		socket.close();


		sleep(2);


		socket.send_to(boost::asio::buffer(reset_msg),receiver_endpoint, 0, ignored_error);
		cout <<"reset the steering!" << endl;


		//lat = double(buffer[0] + buffer[1]*256 + buffer[2]*256*256 + buffer[3]*256*256*256)/10000000-90;
		//lon = double(buffer[4] + buffer[5]*256 + buffer[6]*256*256 + buffer[7]*256*256*256)/10000000-180;
		//hedg = double(buffer[8] + buffer[9]*256)/100;

		//cout << "lat =" << lat << "\n";
		//cout << "lon =" << lon << "\n";
		//cout << "hedg =" << hedg << "\n";


		*/
		//---------------------------------------------------------------------------------------------------------------


		/*
		// socket文件描述符
		int sock_fd;

		// 建立udp socket
		sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
		if(sock_fd < 0)
		{
			perror("socket");
			exit(1);
		}

		// 设置address
		struct sockaddr_in addr_serv;
		int len;
		memset(&addr_serv, 0, sizeof(addr_serv));
		addr_serv.sin_family = AF_INET;
		addr_serv.sin_addr.s_addr = inet_addr(DSET_IP_ADDRESS);
		addr_serv.sin_port = htons(DEST_PORT);
		len = sizeof(addr_serv);


		int send_num;
		int recv_num;
		//char send_buf[20] = "hey, who are you?";
		char send_buf[20] = {103,110, 115, 115, 49}; // ascii "gnss1"
		uint8_t recv_buf[10];


		send_num = sendto(sock_fd, send_buf, strlen(send_buf), 0, (struct sockaddr *)&addr_serv, len);

		if(send_num < 0)
		{
		perror("sendto error:");
		exit(1);
		}
		printf("client send: %s\n", send_buf);


		//recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_serv, (socklen_t *)&len);
		recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, NULL, NULL);


		cout << "recvfrom done" <<"\n";

		if(recv_num < 0)
		{
		perror("recvfrom error:");
		exit(1);
		}

		recv_buf[recv_num] = '\0';
		printf("client receive %d bytes: %s\n", recv_num, recv_buf);

		close(sock_fd);

		*/

		//---------------------------------------------------------------------------------------------------------------



		// subscribe GPS







		//exit(0); // debug -------------------------------------------------------------------------------


		//---------------------------------------------------------------------------------------------------------------


		if (simulation_mode == 1){
			hedg = atan2( wps_y[1]-wps_y[0] , wps_x[1]-wps_x[0] );	        //for simulation
			x0 = wps_x[0] + (wps_x[1]-wps_x[0])/1000.0f; 			//for simulation
			y0 = wps_y[0] + (wps_y[1]-wps_y[0])/1000.0f;			//for simulation
			cout << "simulation mode activated!" << "\n";
		}else{
			lat = gnss_arr[0];						//for testing
			lon = gnss_arr[1];						//for testing
			hedg = hedg_arr[0];						//for testing

			while (lat == 0 || lon == 0)
			{
				lat = gnss_arr[0];
				lon = gnss_arr[1];
				hedg = hedg_arr[0];

				// cout << "1.Waiting......" << endl;
				ros::spinOnce();
			}

			x0 = (lon - lon_ref)*82230.670304;
			y0 = (lat - lat_ref)*111132.944444;
			hedg = hedg2ENU(hedg);// for gnss signal 0, 360 -> -pi, pi
		}
	}

	if (run_mode == 2)
	{
		while (path_loca == false || path_way == false)
		{
			x0 = posx;
			y0 = posy;
			hedg = path_yaw;
			// hedg = hedg_arr[0];
			// hedg = hedg2ENU(hedg);
			
			// cout << "2.Waiting......" << endl;
		}
		path_loca = false;
		path_way = false;
	}	

	cout << "x0" << ":" << x0 << "\n";
	cout << "y0" << ":" << y0 << "\n";
	//cout << "x0" << ":" << x0 << "\n";
	//cout << "y0" << ":" << y0 << "\n";


	// Find closest waypoint index
	d_v2wps_min = 1000;
	for (int j = 0; j < wps_num; j++) {
		x1 = wps_x[j];
		y1 = wps_y[j];
		if(j == wps_num - 1){
			x2 = wps_x[0];
			y2 = wps_y[0];
		}else{
			x2 = wps_x[j+1];
			y2 = wps_y[j+1];
		}

		x0_frt = x0 + d_frt*cos(hedg); 
		y0_frt = y0 + d_frt*sin(hedg);
		ang1 = get_ang(x0, y0, x0_frt, y0_frt, x1, y1, x2, y2); // vehicle heading vector and segment vector angle
		//ang2 = get_ang(x0, y0, x2, y2, x1, y1, x2, y2);// option 2 (but with special case)
		//ang3 = get_ang(x0, y0, x1, y1, x1, y1, x2, y2);// option 2 (but with special case)
		//d_v2seg = find_d_point_2_line ( x0, y0, x1, y1, x2, y2 ); // option 2 (but with special case)
		d_v2wps = sqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1)); //option 1: distance to waypoint

		//cout << "ang1=  " << ang1 << "\n";
		//cout << "ang2=  " << ang2 << "\n";

		if(d_v2wps < d_v2wps_min && ang1 < 1.5708f ){ //see notes for angle 1, 2, 3
			wps_idx = j; // save the closest segment index
			d_v2wps_min = d_v2wps;
		}
	}
	if( d_v2wps_min == 1000 ){
		cout << "ERROR: closest segment index not found !!!!! " << "\n";
		exit(0);
	}else{
		d_v2wps_min = 1000;
		cout << "closest waypoint index: " << wps_idx << "\n";
		cout << "distance to waypoint index " << wps_idx << ": " << d_v2wps << "\n";
	}

	// While loop
	while(ros::ok()){

		pos_up = false;
		ang_up = false;

		cout<<"while loop: Value of variable i is: "<<i << "-------------------------------------------------------------------" <<endl;
		i++;

		// Publish waypoints and lables for rviz visualization==========================================================================

		if(i>0){  // need to publish for 10+ times and then received by rviz, why?
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
			//marker_pub.publish(points);

			// Publish raw waypints using visualization_msgs::Marker::POINTS=============================
			visualization_msgs::Marker points1;// %Tag(MARKER_INIT)%
			points1.header.frame_id = "frame";
			points1.header.stamp = ros::Time::now();
			points1.ns = "waypoints_raw";
			points1.action = visualization_msgs::Marker::ADD;
			points1.pose.orientation.w = 1.0;
			points1.id = 0; // %Tag(ID)%
			points1.type = visualization_msgs::Marker::POINTS; // %Tag(TYPE)%
			points1.scale.x = 0.4;// %Tag(SCALE)% POINTS markers use x and y scale for width/height respectively
			points1.scale.y = 0.4;
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
		visualization_msgs::Marker vehicle_coordinates;
		vehicle_coordinates.header.frame_id = "frame";
		vehicle_coordinates.header.stamp = ros::Time::now();
		vehicle_coordinates.ns = "vehicle_coordinates";
		vehicle_coordinates.action = visualization_msgs::Marker::ADD;
		vehicle_coordinates.pose.orientation.w = 1.0;
		vehicle_coordinates.id = i; // Marker id should be unique
		vehicle_coordinates.type = visualization_msgs::Marker::POINTS;
		vehicle_coordinates.scale.x = 0.2;
		vehicle_coordinates.scale.y = 0.2;
		vehicle_coordinates.color.r = 1.0; //red
		vehicle_coordinates.color.g = 0.0; //
		vehicle_coordinates.color.b = 0.0; //blue
		vehicle_coordinates.color.a = 1.0;
		geometry_msgs::Point p0;
		p0.x = x0;
		p0.y = y0;
		vehicle_coordinates.points.push_back(p0);
		marker_pub.publish(vehicle_coordinates);


		// publish vehicle's propeller_shaft: Marker - line_list ====================================================
		visualization_msgs::Marker line_list;// %Tag(MARKER_INIT)%
		line_list.header.frame_id = "frame";
		line_list.header.stamp = ros::Time::now();
		line_list.ns = "propeller_shaft";
		line_list.action = visualization_msgs::Marker::ADD;
		line_list.pose.orientation.w = 1.0;
		line_list.id = 0; // Marker id should be unique
		line_list.type = visualization_msgs::Marker::LINE_LIST;
		line_list.scale.x = 0.1;// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
		line_list.color.r = 0.0; //red
		line_list.color.g = 0.0; //
		line_list.color.b = 0.0; //blue
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
		box1.color.a = 0.5;
		box2.color.a = 0.5;

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


		//================================================================================




		// Get GPS



		if (run_mode != 2)
		{
			if(simulation_mode != 1){
				lat = gnss_arr[0];
				lon = gnss_arr[1];
				hedg = hedg_arr[0];

				// while (pos_up != true && ang_up != true)
				// {
				// 	lat = gnss_arr[0];
				// 	lon = gnss_arr[1];
				// 	hedg = hedg_arr[0];

				// 	ros::spinOnce();
				// }

				pos_up = false;
				ang_up = false;
				x0 = (lon - lon_ref)*82230.670304; 				//for testing
				y0 = (lat - lat_ref)*111132.944444;				//for testing
				hedg = hedg2ENU(hedg);// for gnss signal 0, 360 -> -pi, pi	//for testing

				cout<< "lat = " << lat <<"\n";
				printf("lat = %0.7f\n", lat);
				cout<< "lon = " << lon <<"\n";

			}
		}
		

		if (path_loca == true)
		{
			x0 = posx;
			y0 = posy;
			hedg = path_yaw;
			// hedg = hedg_arr[0];
			// hedg = hedg2ENU(hedg);

			path_loca = false;
		}

		if (path_way == true)
		{
			// Find closest waypoint index
			d_v2wps_min = 1000;
			for (int j = 0; j < wps_num; j++) {
				x1 = wps_x[j];
				y1 = wps_y[j];
				if(j == wps_num - 1){
					x2 = wps_x[0];
					y2 = wps_y[0];
				}else{
					x2 = wps_x[j+1];
					y2 = wps_y[j+1];
				}

				x0_frt = x0 + d_frt*cos(hedg); 
				y0_frt = y0 + d_frt*sin(hedg);
				ang1 = get_ang(x0, y0, x0_frt, y0_frt, x1, y1, x2, y2); // vehicle heading vector and segment vector angle
				//ang2 = get_ang(x0, y0, x2, y2, x1, y1, x2, y2);// option 2 (but with special case)
				//ang3 = get_ang(x0, y0, x1, y1, x1, y1, x2, y2);// option 2 (but with special case)
				//d_v2seg = find_d_point_2_line ( x0, y0, x1, y1, x2, y2 ); // option 2 (but with special case)
				d_v2wps = sqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1)); //option 1: distance to waypoint

				//cout << "ang1=  " << ang1 << "\n";
				//cout << "ang2=  " << ang2 << "\n";

				if(d_v2wps < d_v2wps_min){// && ang1 < 1.5708f ){ //see notes for angle 1, 2, 3
					wps_idx = j; // save the closest segment index
					d_v2wps_min = d_v2wps;
				}
			}
			if( d_v2wps_min == 1000 ){
				cout << "ERROR: closest segment index not found !!!!! " << "\n";
				exit(0);	
			}else{
				d_v2wps_min = 1000;
				cout << "closest waypoint index: " << wps_idx << "\n";
				cout << "distance to waypoint index " << wps_idx << ": " << d_v2wps << "\n";
			}

			path_way = false;
		}


		//cout<< "hedg =" <<hedg <<"\n";
		//printf("heading = %f\n", hedg);
		//printf("x0: %f, y0: %f\n",x0, y0);



		// Create a waypoint window with waypoint index
		for (int j = 0; j < wps_wdw_num; j++ ){
			int idx_crt = wps_idx - 1 + j;

			if(idx_crt < wps_num && idx_crt >= 0){
				wps_wdw[j] = idx_crt;
			}else if(idx_crt >= wps_num){
				wps_wdw[j] = idx_crt - wps_num;
			}else if(idx_crt < 0){
				wps_wdw[j] = idx_crt + wps_num;
			}
		}

		for(int j = 0; j < wps_wdw_num; j++){
			if(j == 0){cout << "waypoint window: [     ";}
			cout << wps_wdw[j] << "        ";
			if(j == wps_wdw_num -1){cout << "] \n";}
		}

		//cout << "first point in waypoint window: " <<  wps_wdw[0]  <<"\n";		
		//cout << "second point in waypoint window: " <<  wps_wdw[1]  <<"\n";


		// Update closest waypoint index from current one to the next one
		d_v2wps_min = 1000;
		for (int j = 0; j < 3; j++) {
			x1 = wps_x[wps_wdw[j]];
			y1 = wps_y[wps_wdw[j]];
			x2 = wps_x[wps_wdw[j+1]];
			y2 = wps_y[wps_wdw[j+1]];
			x3 = wps_x[wps_wdw[j+2]];
			y3 = wps_y[wps_wdw[j+2]];
			x4 = wps_x[wps_wdw[j+3]];
			y4 = wps_y[wps_wdw[j+3]];
			x5 = wps_x[wps_wdw[j+4]];
			y5 = wps_y[wps_wdw[j+4]];
			x6 = wps_x[wps_wdw[j+5]];
			y6 = wps_y[wps_wdw[j+5]];
			cout << "x4=" << x4 << "; y4=" << y4 << "\n";
			cout << "x5=" << x5 << "; y5=" << y5 << "\n";
			cout << "x6=" << x6 << "; y6=" << y6 << "\n";


			x0_frt = x0 + d_frt*cos(hedg); 
			y0_frt = y0 + d_frt*sin(hedg);
			//ang1 = get_ang(x0, y0, x0_frt, y0_frt, x1, y1, x2, y2); // vehicle heading vector and segment vector angle
			//ang2 = get_ang(x0, y0, x2, y2, x1, y1, x2, y2); // option 2 (but with special case)
			//ang3 = get_ang(x0, y0, x1, y1, x1, y1, x2, y2); // option 2 (but with special case)
			//d_v2seg = find_d_point_2_line ( x0, y0, x1, y1, x2, y2 ); // option 2 (but with special case)
			d_v2wps = sqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1)); //option 1: find distance to waypoint
			//cout << "ang1=  " << ang1 << "\n";
			//cout << "ang2=  " << ang2 << "\n";
			//cout << "ang3=  " << ang3 << "\n";
			//cout << "distance to waypoint index " << wps_wdw[j] << "-" << wps_wdw[j+1] <<":" << d_v2wps << "\n";


			if(d_v2wps < d_v2wps_min ){ 
				wps_idx = wps_wdw[j]; // update the closest global waypoint index
				d_v2wps_min = d_v2wps;
				wps_wdw_idx = j; // update window waypoint index
			}

			//cout<< "d v 2 wps =" <<d_v2wps <<"\n";
			//cout<< "d v 2 wps min =" <<d_v2wps_min <<"\n";			
		}
		if( d_v2wps_min == 1000 ){
			cout << "ERROR: closest segment index not found !!!!! " << "\n";
			exit(0);	
		}else{
			cout << "closest waypoint index: [ ...... " << wps_wdw[wps_wdw_idx]  << " ...... ] \n";
			cout << "distance to waypoint index " << wps_idx << ": " << d_v2wps_min << "\n";
		}

		d_v2wps_min = 1000;


	

		// Find r / curvature ----------------------------------------------------------------------------------------------
		x1 = wps_x[wps_wdw[wps_wdw_idx]];
		y1 = wps_y[wps_wdw[wps_wdw_idx]];
		x2 = wps_x[wps_wdw[wps_wdw_idx+1]];
		y2 = wps_y[wps_wdw[wps_wdw_idx+1]];
		x3 = wps_x[wps_wdw[wps_wdw_idx+2]];
		y3 = wps_y[wps_wdw[wps_wdw_idx+2]];
		cout << "x1:" << x1 << " y1:"<<y1<<"\n"<<"x2:"<<x2<<" y2:"<<y2<<"\n"<<"x3:"<<x3<<" y3:"<<y3<<"\n";
		get_r_center (x1, y1, x2, y2, x3, y3, r, cx0, cy0);
		//cout << "cx0:" << cx0 <<", " << "cy0:" << cy0 << "," << "r:" << r << "\n";

		// Determine steering control mode ---------------------------------------------------------------------------------
		if(r>-50 && r<=0 || r>0 && r<50){
			str_ctrl_mode = 0;

		}else{
			str_ctrl_mode = 1;

		}

		//str_ctrl_mode = 2; //  2.MPC 3. pure pursuit --------------------------------------------------------------------------------> change mode manually!


		// Run steering controller (switch case/ str mode) -------------------------------------------------------------------
		switch(str_ctrl_mode){

			case 0: // 3pts curv pid
				{
				cout << "3pts curve tracking PID mode is activated now!" << "\n";
				d_o2v = sqrt((cx0-x0_frt)*(cx0-x0_frt)+(cy0-y0_frt)*(cy0-y0_frt));
				offset_ctrl = abs(r) - d_o2v;
				if (r < 0){
					offset_ctrl = - offset_ctrl;
				}
				Kp = 60; // P gain: 1000
				d_offset = offset_ctrl - offset_ctrl_pre;
				offset_ctrl_pre = offset_ctrl;
				if ( abs(offset_ctrl) > 0.3 ){
					Kd = 100; // D gain: 3000
				}else{
					Kd = 30; // D gain: 1000
				}
				str_wheel_ang_ref = VL22_model_r2str(r);

				cout << "str_wheel_ang_ref = " << str_wheel_ang_ref << "\n";

				str_wheel_ang = str_wheel_ang_ref + Kp*offset_ctrl + Kd*d_offset;


				cout << "r=" << r << "; d_o2v=" << d_o2v << "; offset=" << offset_ctrl << "; d_offset=" << d_offset << "\n";
				cout << "str_wheel_ang_ref =" << str_wheel_ang_ref << "; str_wheel_ang =" << str_wheel_ang << "\n";

				break;
				}

			case 1: // 2pts straight pid
				{
				cout << "2pts straight line tracking PID mode is activated now" << "\n";
				d_p2line = find_d_point_2_line( x0_frt, y0_frt, x1, y1, x2, y2 );
				side2line = find_side_2_line( x0_frt, y0_frt, x1, y1, x2, y2 );
				offset_ctrl = d_p2line * side2line;
				if ( abs(offset_ctrl) > 1 ){
					Kp = 100; // 500
				}else{
					Kp = 80; // 300
				}
				d_offset = offset_ctrl - offset_ctrl_pre;
				offset_ctrl_pre = offset_ctrl;
				if ( abs(offset_ctrl) > 0.3 ){
					Kd = 250; // 3000
				}else{
					Kd = 500; // 1000
				}
				str_wheel_ang_ref = 0;

				cout << "str_wheel_ang_ref = " << str_wheel_ang_ref << "\n";

				str_wheel_ang = str_wheel_ang_ref + Kp*offset_ctrl + Kd*d_offset;
				cout << "offset=" << offset_ctrl << "; str_wheel_ang =" << str_wheel_ang << "\n";
				break;
				}

			case 2:// udacity MPC
				{
				cout << "MPC mode is activated now!" << "\n";				
				vector<double> ptsx {wps_x[wps_wdw[wps_wdw_idx]], wps_x[wps_wdw[wps_wdw_idx+1]], wps_x[wps_wdw[wps_wdw_idx+2]], wps_x[wps_wdw[wps_wdw_idx+3]], wps_x[wps_wdw[wps_wdw_idx+4]], wps_x[wps_wdw[wps_wdw_idx+5]], wps_x[wps_wdw[wps_wdw_idx+6]], wps_x[wps_wdw[wps_wdw_idx+7]], wps_x[wps_wdw[wps_wdw_idx+8]], wps_x[wps_wdw[wps_wdw_idx+9]], wps_x[wps_wdw[wps_wdw_idx+10]], wps_x[wps_wdw[wps_wdw_idx+11]]};
				vector<double> ptsy {wps_y[wps_wdw[wps_wdw_idx]], wps_y[wps_wdw[wps_wdw_idx+1]], wps_y[wps_wdw[wps_wdw_idx+2]], wps_y[wps_wdw[wps_wdw_idx+3]], wps_y[wps_wdw[wps_wdw_idx+4]], wps_y[wps_wdw[wps_wdw_idx+5]], wps_y[wps_wdw[wps_wdw_idx+6]], wps_y[wps_wdw[wps_wdw_idx+7]], wps_y[wps_wdw[wps_wdw_idx+8]], wps_y[wps_wdw[wps_wdw_idx+9]], wps_y[wps_wdw[wps_wdw_idx+10]], wps_y[wps_wdw[wps_wdw_idx+11]]};
				
			        //vector<double> ptsx {wps_x[wps_wdw[wps_wdw_idx]], wps_x[wps_wdw[wps_wdw_idx+1]], wps_x[wps_wdw[wps_wdw_idx+2]], wps_x[wps_wdw[wps_wdw_idx+3]]};
				//vector<double> ptsy {wps_y[wps_wdw[wps_wdw_idx]], wps_y[wps_wdw[wps_wdw_idx+1]], wps_y[wps_wdw[wps_wdw_idx+2]], wps_y[wps_wdw[wps_wdw_idx+3]]};
				
				double px = x0;
				double py = y0;
				double psi = hedg;
				double v = 3; // 0.89m/s=2mph


				double steer_value;
				double throttle_value;
				vector<double> mpc_x_v;
				vector<double> mpc_y_v;

				vector<double> next_x_v(ptsx.size());
				vector<double> next_y_v(ptsx.size());




				vector<double> x_ref(11);
				vector<double> y_ref(11);



				try
				{
					mpc_fun(ptsx, ptsy, px, py, psi, v, steer_value, throttle_value, mpc_x_v, mpc_y_v, next_x_v, next_y_v, x_ref, y_ref);
				}
				catch(const std::exception& e)
				{
					cout << "mpc function error......." << '\n';
				}
				mpc_x_vals = mpc_x_v;
				mpc_y_vals = mpc_y_v;



				next_x_vals = next_x_v;
				next_y_vals = next_y_v;

				mpc_x_ref = x_ref; 
				mpc_y_ref = y_ref;



				//str_ang = mpc_func(......);
				//str_step = str_ang(udacity)/25000*720??
				cout << "steer_value =" << steer_value << "\n";
				str_step = steer_value / deg2rad(25);
				str_step = 25000*str_step;





				cout << "size next_x_vals =" << next_x_vals.size() << endl; 
				cout << "size mpc_x_ref =" << mpc_x_ref.size() << endl;  // = 0  ???!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

				//exit(0);





				break;
				}

			case 3://pure pursuit	
				{
				cout << "pure pursuit mode is activated now!" << "\n";
				d_o2v = sqrt((cx0-x0_frt)*(cx0-x0_frt)+(cy0-y0_frt)*(cy0-y0_frt));
				offset_ctrl = abs(r) - d_o2v;
				if (r < 0){
					offset_ctrl = - offset_ctrl;
				}

				str_step = trj_plan_pp( x0, y0, hedg, x1, y1, x2, y2 );
				}

			
		}




		//cout << "size next_x_vals =" << next_x_vals.size() << endl; 
		//cout << "size mpc_x_ref =" << mpc_x_ref.size() << endl;  // = 0  ???!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!




		// Transmit steering cmd ----------------------------------------------------------------------------------------------------------

		/*
		// +++++++++ crew cab 905 testing +++++++++++++++++

		str_cmd	= -str_step; // 0828 larger steering angle // use neg sign to match mbed
				     // send an array with 20 elements to Mbed

		cout << "str_cmd =" << str_cmd << "\n";

		int str_cmd_int = int(str_cmd);

		int str_cmd_string[2];

		str_cmd_string[0] = (str_cmd_int >> 8) & 0xFF;
		str_cmd_string[1] = str_cmd_int & 0xFF;
		cout << "str_cmd_string [0] =" << str_cmd_string[0] << "\n";
		cout << "str_cmd_string [1] =" << str_cmd_string[1] << "\n";

		std_msgs::Int16 str_msg;

		//str_msg.data = os.str();
		str_msg.data = str_cmd_int;

		//float_pub.publish(str_msg);
		str_pub.publish(str_msg);

		*/
		

		// ++++++++++ vl22 22787 testing +++++++++++
		
		
		std_msgs::Int16MultiArray str_cmd_msg; // type: std_msgs::Int16 / topic name: "str_wheel_ang_cmd"


		int16_t str_wheel_ang_cmd = int16_t(str_wheel_ang);

	cout << "str_wheel_ang_cmd" << str_wheel_ang_cmd<<"\n";

          cout << "mpc debug 1" << "\n";
		str_cmd_msg.data.clear();
		str_cmd_msg.data.push_back(str_wheel_ang_cmd);

          cout << "mpc debug 2" << "\n";

		str_pub.publish(str_cmd_msg);

		std_msgs::Int16 str_msg;
		str_msg.data = -str_wheel_ang_cmd;
		str_IPG_pub.publish(str_msg);

          cout << "mpc debug 3" << "\n";






		//exit(0); // debug -------------------------------------------------------------------------------




		// Publish the error ============================================================================================
		std_msgs::Float64 error;
		error.data = offset_ctrl;
		float_pub.publish(error);

		// publish planned trjectory by mpc
		theta1 = atan2(y1 - cy0, x1 - cx0); // find the starting point based on the parametric equation of the circle
		theta3 = atan2(y3 - cy0, x3 - cx0); // find the starting point based on the parametric equation of the circle
		if(theta3 - theta1 > 3.1415926){
			theta3 = theta3 - 2*3.1415926;
		}else if(theta3 - theta1 < -3.1415926){
			theta3 = theta3 + 2*3.1415926;
		}

		visualization_msgs::Marker traj;
		traj.header.frame_id = "frame";
		traj.header.stamp = ros::Time::now();
		traj.ns = "trajectory";
		traj.action = visualization_msgs::Marker::ADD;
		traj.id = 0; // Marker id should be unique
		traj.type = visualization_msgs::Marker::POINTS;
		traj.scale.x = 0.5;
		traj.scale.y = 0.5;
		traj.scale.z = 0.5;
		traj.color.r = 0.0;
		traj.color.g = 0.0;
		traj.color.b = 1.0; //blue
		traj.color.a = 1.0;

		if(str_ctrl_mode == 0){
			for(int j=0;j<21;j++){
				geometry_msgs::Point p2;
				p2.x = cx0 + abs(r)*cos(theta1 + j*(theta3 - theta1)/20.0f);
				p2.y = cy0 + abs(r)*sin(theta1 + j*(theta3 - theta1)/20.0f);
				traj_x[j] = p2.x;
				traj_y[j] = p2.y;

				//cout<<"theta1="<<theta1<<"; theta3="<<theta3<<"\n";
				//cout<<"traj_x = " << p2.x <<"; traj_y = " << p2.y << "\n";
				traj.points.push_back(p2);

			}
		}else if(str_ctrl_mode == 1){
			for(int j=0;j<21;j++){
				geometry_msgs::Point p2;
				p2.x = x1 + j*(x3 - x1)/20.0f;
				p2.y = y1 + j*(y3 - y1)/20.0f;
				traj_x[j] = p2.x;
				traj_y[j] = p2.y;
				traj.points.push_back(p2);
			}
		}else if(str_ctrl_mode == 2){
			for(int j=0;j<mpc_x_vals.size();j++){
				geometry_msgs::Point p2;
				p2.x = x0+mpc_x_vals[j]*cos(-hedg)+mpc_y_vals[j]*sin(-hedg);
				p2.y = y0-mpc_x_vals[j]*sin(-hedg)+mpc_y_vals[j]*cos(-hedg);
				traj_x[j] = p2.x;
				traj_y[j] = p2.y;
				traj.points.push_back(p2);
				// cout << "mpc_x_vals[" << j << "]]=" << traj_x[j] << "; mpc_y_vals[" << j << "]]=" << traj_y[j] << '\n';
			}
			cout << "mpc.size = " << mpc_x_vals.size() <<  '\n';
		}


		marker_pub.publish(traj);



		// publish reference waypoints (for poly-fit)
		visualization_msgs::Marker refer;
		refer.header.frame_id = "frame";
		refer.header.stamp = ros::Time::now();
		refer.ns = "reference";
		refer.action = visualization_msgs::Marker::ADD;
		refer.id = 0; // Marker id should be unique
		refer.type = visualization_msgs::Marker::POINTS;
		refer.scale.x = 0.5;// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
		refer.scale.y = 0.5;
		refer.scale.z = 0.5;
		refer.color.r = 0.0; //red
		refer.color.g = 1.0; //green
		refer.color.a = 1;
		if(str_ctrl_mode == 2){
			for(int j=0;j<next_x_vals.size();j++){
				geometry_msgs::Point p3;
				p3.x = x0+next_x_vals[j]*cos(-hedg)+next_y_vals[j]*sin(-hedg);
				p3.y = y0-next_x_vals[j]*sin(-hedg)+next_y_vals[j]*cos(-hedg);
				refer.points.push_back(p3);
				marker_pub.publish(refer);
			}
		}

		if (run_mode == 2)
		{
			// publish reference waypoints (for poly-fit) 
			visualization_msgs::Marker refer;
			refer.header.frame_id = "/frame";
			refer.header.stamp = ros::Time::now();
			refer.ns = "reference";
			refer.action = visualization_msgs::Marker::ADD;
			refer.id = 0; // Marker id should be unique
			refer.type = visualization_msgs::Marker::POINTS;
			refer.scale.x = 0.5;// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
			refer.scale.y = 0.5;
			refer.scale.z = 0.5;
			refer.color.r = 0.0; //red
			refer.color.g = 1.0; //green
			refer.color.a = 1;
			for(int j=0;j<wps_num;j++){
				geometry_msgs::Point p3;
				p3.x = wps_x[j];
				p3.y = wps_y[j];
				refer.points.push_back(p3);					
			}
			marker_pub.publish(refer);
		}

		// publish reference trajectory fitting result
		visualization_msgs::Marker reference_trajectory_fitting;
		reference_trajectory_fitting.header.frame_id = "frame";
		reference_trajectory_fitting.header.stamp = ros::Time::now();
		reference_trajectory_fitting.ns = "reference_trajectory_fit";
		reference_trajectory_fitting.action = visualization_msgs::Marker::ADD;
		reference_trajectory_fitting.pose.orientation.w = 1.0;
		reference_trajectory_fitting.id = 0; // Marker id should be unique
		reference_trajectory_fitting.type = visualization_msgs::Marker::LINE_STRIP;
		reference_trajectory_fitting.scale.x = 0.8;// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
		reference_trajectory_fitting.color.r = 0.0; //red
		reference_trajectory_fitting.color.g = 1.0; //green
		reference_trajectory_fitting.color.b = 0.0;
		reference_trajectory_fitting.color.a = 0.7;

		if(str_ctrl_mode == 2){
			for(int j=0;j<mpc_x_ref.size();j++){
				geometry_msgs::Point p4;
				p4.x = x0+mpc_x_ref[j]*cos(-hedg)+mpc_y_ref[j]*sin(-hedg);
				p4.y = y0-mpc_x_ref[j]*sin(-hedg)+mpc_y_ref[j]*cos(-hedg);
				reference_trajectory_fitting.points.push_back(p4);
			}
			marker_pub.publish(reference_trajectory_fitting);
		}


		/*
		// debug: publish the closest waypoints
		visualization_msgs::Marker closest_waypoints;
		closest_waypoints.header.frame_id = "frame";
		closest_waypoints.header.stamp = ros::Time::now();
		closest_waypoints.ns = "closest_waypoints";
		closest_waypoints.action = visualization_msgs::Marker::ADD;
		closest_waypoints.pose.orientation.w = 1.0;
		closest_waypoints.id = 0; // Marker id should be unique
		closest_waypoints.type = visualization_msgs::Marker::LINE_STRIP;
		closest_waypoints.scale.x = 1.0;// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
		closest_waypoints.color.r = 1.0; //red
		closest_waypoints.color.g = 1.0; //green
		closest_waypoints.color.a = 1.0;

		if(str_ctrl_mode == 2){
			geometry_msgs::Point p5;
			p5.x = x0+x1*cos(-hedg)+y1*sin(-hedg);
			p5.y = y0-x1*sin(-hedg)+y1*cos(-hedg);
			reference_trajectory_fitting.points.push_back(p5);
			p5.x = x0+x2*cos(-hedg)+y2*sin(-hedg);
			p5.y = y0-x2*sin(-hedg)+y2*cos(-hedg);
			reference_trajectory_fitting.points.push_back(p5);

			marker_pub.publish(closest_waypoints);
		}

		*/



		//debug==============================================================================================================

		float r2 = sqrt((x1-cx0)*(x1-cx0)+(y1-cy0)*(y1-cy0));
		float r3 = sqrt((x3-cx0)*(x3-cx0)+(y3-cy0)*(y3-cy0));
		cout <<"r="<< r <<"; r2="<<r2<<"; r3="<<r3<<"\n";


		/*
		if(abs(theta1-theta3)>3.1415926){
			exit(0);
		}

		if((traj_x[0] - x1) > 2 || (traj_y[0] - y1) > 2){
			exit(0);
		}

		if( offset_ctrl > 20 || offset_ctrl < -20 ){
			exit(0);
		}
		*/

		//if(wps_idx == 38){
		//	exit(0);
		//}






		//simulation =========================================================================================================

		if(simulation_mode == 1){
			int switch_model = 0; // 0: kinematic, 1: lateral dynamic
			switch(switch_model){
				//case 0: MKZ_model ( x0, y0, hedg, str_step, d_travel, add_x0, add_y0, add_hedg );
				case 0: truck_kinematics_model( x0, y0, hedg, str_step, d_travel, add_x0, add_y0, add_hedg );
				case 1: dynamic_model ( x0, y0, hedg, str_step, d_travel, add_x0, add_y0, add_hedg );
			}
			x0 = add_x0;
			y0 = add_y0;
			hedg = add_hedg;
			x0_frt = x0 + d_frt*cos(hedg); // ???????????????????????????????????????????????????????????????????????????????   cos???   sin???
			y0_frt = y0 + d_frt*sin(hedg);
			//cout << "x0 next:" << x0 << "    ";//simulation
			//cout << "y0 next:" << y0 << "    ";//simulation
			//cout << "hedg next:" << hedg/3.14f*180.0f << "\n";//simulation
			//cout << "x0_frt next:" << x0_frt << "    ";//simulation
			//cout << "y0_frt next:" << y0_frt << "\n";//simulation
			cout << "--------------------------------------------------->>>>>>  str_step = " <<  str_step << "\n";
		}

		// ros::spinOnce();

		rate.sleep();
	}
	return 0;


}
