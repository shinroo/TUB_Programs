cplusplus {{
#include "../common/protsim_defines.h"
}}

message NetworkPacket
{
    fields:
	string baseName;
	long id;
	long srcNode = PS_NODE_UNKNOWN;
	long srcAppl = PS_APP_UNKNOWN;
	long destNode = PS_NODE_UNKNOWN;
	long destAppl = PS_APP_UNKNOWN;
	bool alert = false;
	long lastNode = PS_NODE_UNKNOWN;
	long TTL = 0;
}
