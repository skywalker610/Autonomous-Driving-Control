#include <kvaser.h>

kvaser::kvaser(){}

void kvaser::kvaserinit(){
   	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) perror("Socket error");
	strcpy(ifr.ifr_name, "can0" );
	ioctl(s, SIOCGIFINDEX, &ifr);
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) perror("Bind error");    
}

void kvaser::kvaserread(){
	nbytes = read(s, &frame, sizeof(struct can_frame));
    if (nbytes < 0) perror("Read error");
}

void kvaser::kvaserwrite(){
	if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) perror("Write error");
}

void kvaser::kvaserclose(){
	if(close(s)<0) perror("Close");
}