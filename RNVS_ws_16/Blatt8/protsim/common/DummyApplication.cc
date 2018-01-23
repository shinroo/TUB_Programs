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

#include <omnetpp.h>
#include "protsim_defines.h"
#include "Application.h"

/**
 *  Dummy application. Works as a place holder if an application is not
 *  needed.
 */
class DummyApplication : public Application {
public:
    Module_Class_Members(DummyApplication,Application,0);

protected:
    virtual void initialize(); ///< Sets applAddr

    /** As DummyApplication is only a placeholder, handleMessage should never be called.
     *  The simulation will be aborted if it is called.
     */
    virtual void handleMessage(cMessage * msg) { abort(); }
};

Define_Module_Like(DummyApplication,Application);

void DummyApplication::initialize() {
    par("applAddr").setLongValue(PS_DUMMY_APP);
    Application::initialize();   
}
