/***************************************************************************
 *
 * This file is part of the ProtSim framework developed by TKN for a
 * practical course on basics of simulation and Internet protocol functions
 *
 * Copyright:   (C)2004-2007 Telecommunication Networks Group (TKN) at
 *              Technische Universitaet Berlin, Germany.
 *
 * Authors:     Lars Westerhoff, Guenter Schaefer
 *
 **************************************************************************/

#include "ExternalApplication.h"

void ExternalApplication::sendExternal(ProtSimSocketRTScheduler::DataChunk* data) {
    ProtSimSocketRTScheduler* scheduler = check_and_cast<ProtSimSocketRTScheduler*>(simulation.scheduler());
    scheduler->sendBytes(this, data);
}
