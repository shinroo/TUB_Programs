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

#include <cassert>
#include "Application.h"

void Application::initialize() {
    networkStack = dynamic_cast<NetworkStack*>(gate("out")->destinationGate()->ownerModule());
    assert(networkStack);
    char n[128];
    snprintf(n,sizeof(n),"%s%ld",className(),par("applAddr").longValue());
    setName(n);
}
