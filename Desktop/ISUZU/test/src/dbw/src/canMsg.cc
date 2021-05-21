#include "canMsg.h"

canMsg::canMsg(){ kva.kvaserinit();} // Initiate can bus message system

void canMsg::can_message_read() { kva.kvaserread(); std::cout<<"调用成功！"<<std::endl;}

void canMsg::can_message_write() { kva.kvaserwrite();}

void canMsg::can_message_close() { kva.kvaserclose();}

can_frame canMsg::canFrame(){return kva.frame;}
