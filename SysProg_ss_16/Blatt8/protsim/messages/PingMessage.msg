cplusplus {{
#include "NetworkPacket_m.h"
}}

class NetworkPacket;

message PingMessage extends NetworkPacket
{
    fields:
	long       seqNo = -1;
	bool       reply = false;
}
