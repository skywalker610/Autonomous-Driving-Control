Control for VL22

# This code is for drive by wire control of VL22787, includes drive by wire and control parts. All locate in control directory.

Compile 

under Control-AD $ catkin_make

Run

Under Control-AD  $ source devel/setup.bash
		   $ roslaunch dbw dbw_control.launch
	
		   
Note: 1 dbw_control launch the gnss node, sbw_send node and truck_node node, receiving the gps data, control the steering wheel and support control algorithm
      2 gps_common_msgs and hellocm_msgs are dependence files for truck.cpp, path_calib is for calibration of control, path_draw_Google can support drawing path on Google with given longitude and latitude data, path_recording is for recording gps data of waypoints.
