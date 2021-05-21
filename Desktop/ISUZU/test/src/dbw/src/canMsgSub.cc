#include <canMsgSub.h>

canMsgSub::canMsgSub():canMsg(){
  ros::Publisher GNSSmsg_pub = n.advertise<std_msgs::Float64MultiArray>("gnss_msg", 3);
  ros::Publisher ACCEmsg_pub = n.advertise<std_msgs::Float32MultiArray>("acc_msg", 3);
  ros::Publisher GYROmsg_pub = n.advertise<std_msgs::Float32MultiArray>("gyr_msg", 3);
  ros::Publisher ANGLmsg_pub = n.advertise<std_msgs::Float32MultiArray>("ang_msg", 3);
  ros::Publisher ALTImsg_pub = n.advertise<std_msgs::Float32>("alt_msg", 3);
  ros::Publisher GPSmsg_pub = n.advertise<dbw::GPSFix>("gpsfix_msg", 3);
  ros::Publisher IMUmsg_pub = n.advertise<sensor_msgs::Imu>("imu_msg", 3);
  ros::Publisher NAVImsg_pub = n.advertise<sensor_msgs::NavSatFix>("nav_msg", 3);
  ros::Publisher Speed_pub = n.advertise<std_msgs::Float32>("speed_msg", 3);
}

void canMsgSub::readGpsImudata() {
  while(true){
    canMsgSub::can_message_read();
    struct can_frame frame = canMsgSub::canFrame();
    switch (frame.can_id) {
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
    if (gyr_xy_up && gyr_z_up){
            gyr_xy_up = false;
            gyr_z_up = false;
            gyr_up = true;
        }
	  if (acc_xy_up && acc_z_up){
            acc_xy_up = false;
            acc_z_up = false;
            acc_up = true;
        }
  }
    
}

void canMsgSub::readSpeed() {
  canMsgSub::can_message_read();
  struct can_frame frame = canMsgSub::canFrame();
  if (frame.can_id == (0x18fef100 | 0x80000000) ){     //read vehicle speed id
		uint16_t vehicle_speed_bytes = frame.data[2] | frame.data[1] << 8;
		vehicle_speed.data = vehicle_speed_bytes / 256.0;
	}
}
//TO DO
void canMsgSub::run() {
  readGpsImudata();
  readSpeed();
  canMsgSub::can_message_close();
  std::ofstream outfile; // Save GPS data
  outfile.open("position.dat");
  std::ofstream accfile;
  accfile.open("acceleration.dat");
  ros::Rate loop_rate(50);
	while (ros::ok()) {
    posArray.data.clear();
    accArray.data.clear();
    gyrArray.data.clear();
    angArray.data.clear();
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
            outfile << std::fixed << std::setprecision(9) << pos[1]<< std::endl;
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
            accfile << std::fixed << std::setprecision(4) << acc[2] << std::endl;
        }
    }
    Speed_pub.publish(vehicle_speed);
		ros::spinOnce();
		loop_rate.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "canMsgSub");
  canMsgSub canmsgsub;
  canmsgsub.run();
  return 0;
}
