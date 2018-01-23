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

#ifndef METRIC_FUNC_H
#define METRIC_FUNC_H

#include <omnetpp.h>
#include "protsim_defines.h"

class NetworkStack;

/**
 * Abstract base class functor (function object) for the calculation
 * of link metrics (costs).
 */
class MetricFunc : public cObject {
public:
    /// \name Overridden methods from cObject
    //@{
    virtual DUP_RETVAL * dup() const;
#ifdef OMNET30
    virtual std::string info();
#else
    virtual void info(char *buf);
#endif
    virtual void writeContents(std::ostream& os);
    virtual const char * className() const = 0;
    //@}

    /** 
     * Calculate metric
     * 
     * @param networkStack 
     * @param outgate Gate to external link.
     * @return metric of external link.
     */
    virtual double getMetric(const NetworkStack * networkStack,
			     cGate * outgate) = 0;
};

#endif /* METRIC_FUNC_H */
