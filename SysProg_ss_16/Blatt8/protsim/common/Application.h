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

#ifndef APPLICATION_H
#define APPLICATION_H

#include <omnetpp.h>
#include "NetworkStack.h"

/**
 * Abstract base class for application (user applications and routing daemons).
 * Applications reside in a NetworkNode and are connected to a NetworkStack.
 * An application is uniquely identified with in a NetworkNode by its
 * parameter \c applAddr.
 */
class Application : public cSimpleModule {
protected:
    /**
     * Constructor called by sub classes
     *
     * @param name Name of the application
     * @param parentmod Parent module (usually the NetworkNode)
     * @param stack Stack size of coroutine
     */
    Application(const char * name = NULL, cModule * parentmod = NULL,
                unsigned stack = 8192)
	: cSimpleModule(name,parentmod,stack), networkStack(NULL) { }

    /// initializes networkStack and name (class name concatenated by \c applAddr)
    virtual void initialize();

protected:
    NetworkStack * networkStack; ///< Pointer to the NetworkStack in the same node.
};

#endif /* APPLICATION_H */
