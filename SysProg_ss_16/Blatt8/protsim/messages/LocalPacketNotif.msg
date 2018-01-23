cplusplus {{
#include "NetworkPacket_m.h"
}}

class NetworkPacket;

message LocalPacketNotif extends NetworkPacket
{
    fields:
	bool         noRoute;
}
