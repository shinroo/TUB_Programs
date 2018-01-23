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

#ifndef EXTERNAL_APPLICATION_H
#define EXTERNAL_APPLICATION_H

#include <omnetpp.h>
#include "Application.h"
#include "NetworkStack.h"
#include "../emulation/ProtSimSocketRTScheduler.h"

/**
 * Abstract base class for proxies to external applications.
 */
class ExternalApplication : public Application {
protected:
    /**
     * Constructor called by sub classes
     *
     * @param name Name of the application
     * @param parentmod Parent module (usually the NetworkNode)
     * @param stack Stack size of coroutine
     */
    ExternalApplication(const char * name = NULL, cModule * parentmod = NULL,
			unsigned stack = 8192)
	: Application(name,parentmod,stack) { }

    void sendExternal(ProtSimSocketRTScheduler::DataChunk* data);
};

#endif /* EXTERNAL_APPLICATION_H */
