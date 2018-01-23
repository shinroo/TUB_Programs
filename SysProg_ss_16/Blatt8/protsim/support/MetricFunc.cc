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

#include "MetricFunc.h"

DUP_RETVAL * MetricFunc::dup() const {
    copyNotSupported();
    return NULL;
}

#ifdef OMNET30
std::string MetricFunc::info() {
    return cObject::info() + std::string(" ") + std::string(className());
}
#else
void MetricFunc::info(char *buf) {
    cObject::info( buf );

    sprintf( buf+strlen(buf), " %s",className());
}
#endif

void MetricFunc::writeContents(std::ostream& os) {
    os << " " << className() << endl;
}
