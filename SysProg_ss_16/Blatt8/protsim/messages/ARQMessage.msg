cplusplus {{
#include "NetworkPacket_m.h"
}}

class NetworkPacket;

message ARQData extends NetworkPacket
{
    fields:
	long       seqNo = -1;
}

message ARQAck extends NetworkPacket
{
    fields:
	long       seqNoExpected = -1;
}
