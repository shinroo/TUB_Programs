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
#include "DefaultMetric.h"
#include "NetworkStack.h"

Register_Class(DefaultMetric);

const char * DefaultMetric::className() const {
    return "DefaultMetric";
}

double DefaultMetric::getMetric(const NetworkStack * networkStack,
				cGate * outgate) {
    double retval = 1.0;

    cChannel * ch = networkStack->getExternalLink(outgate);
    if (ch->hasPar("cost")) 
	retval = ch->par("cost").doubleValue();
    if (retval <= 0.0) retval = 1.0;

    return retval;
}
