#ifndef _HTTP_H_
#define _HTTP_H_
#include "socket.h"
void Serialize_HttpReport(u8* buf,Packet_connectData data,u8* buflen);
u8 transport_HttpSendPacketBuffer(u8* server_addr,u8* port,u8* buf,u8 buflen);
#endif
