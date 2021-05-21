#ifndef KVASER_H
#define KVASER_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

class kvaser{
  public:
    kvaser();
	  void kvaserinit(); // kvaser read interface
    void kvaserread(); // read data from can bus
    void kvaserwrite(); // write data into can busv
    void kvaserclose();
    friend class canMsg; // class canMsg can access kvaser private
  private:
    int s;
	  int nbytes;
	  struct sockaddr_can addr; // socketCAN
	  struct ifreq ifr;
	  struct can_frame frame;

};

#endif //KVASER_H
