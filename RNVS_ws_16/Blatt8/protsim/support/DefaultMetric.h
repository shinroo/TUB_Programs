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

#ifndef DEFAULT_METRIC_H
#define DEFAULT_METRIC_H

#include "MetricFunc.h"

/**
 * Metric that just returns the \c cost parameter of a link.
 */
class DefaultMetric: public MetricFunc {
public:
    virtual const char * className() const;

    /** 
     * Calculate metric by reading the \c cost parameter
     * If there is no such parameter, 1.0 will be returned, resulting in a
     * hop count metric.
     *
     * @param networkStack 
     * @param outgate Gate to external link.
     * @return metric of external link.
     */
    virtual double getMetric(const NetworkStack * networkStack,
			     cGate * outgate);
};


#endif /* DEFAULT_METRIC_H */
