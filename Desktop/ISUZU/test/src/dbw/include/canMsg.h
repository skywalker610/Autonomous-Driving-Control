#ifndef CANMSG_H
#define CANMSG_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt8MultiArray.h"
#include <kvaser.h>

class canMsg{
    public:
    canMsg();
    kvaser kva;
    void can_message_read();
    void can_message_write();
    void can_message_close();
    can_frame canFrame();
};

#endif //CANMSG_H