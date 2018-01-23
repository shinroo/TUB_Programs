cplusplus {{
#include "NetworkPacket_m.h"
}}

class NetworkPacket;

message LocalLinkChangeNotif extends NetworkPacket
{
    fields:
	unsigned int linkIndex;
	long         nextHop;
	double       newCost;
	double       oldCost;
}
