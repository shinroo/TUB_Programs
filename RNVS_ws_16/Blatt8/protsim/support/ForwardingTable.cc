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

#include <iomanip>
#include "ForwardingTable.h"

const char * ForwardingTable::className() const {
    return "ForwardingTable";
}

DUP_RETVAL * ForwardingTable::dup() const {
    copyNotSupported();
    return NULL;
}

#ifdef OMNET30
std::string ForwardingTable::info() {
    if (table.empty())
	return cObject::info() + std::string(" (empty)");
    else {
	char d[10];
	snprintf(d,sizeof(d)," (size=%d)", table.size());
	return cObject::info() + std::string(d);
    }
}
#else
void ForwardingTable::info(char *buf) {
    cObject::info( buf );

    if (table.empty())
        sprintf( buf+strlen(buf), " (empty)" );
    else
        sprintf( buf+strlen(buf), " (size=%d) ", table.size());
}
#endif

void ForwardingTable::writeContents(std::ostream& os) const {
    if (table.empty()) {
	os << "  (empty)\n";
    }
    else {
	for (DestinationMapT::const_iterator i = table.begin();
	     i != table.end(); ++i) {
	    os << "  " << std::setw(5) << i->first << " --> " << i->second->name();
	    if (i->second->isVector()) {
		os << "[" << i->second->index() << "]";
	    }
	    os << "\n";
	}
    }
    os.flush();
}
 
void ForwardingTable::clear() {
    table.clear();
}

void ForwardingTable::add(NodeAddressT dest, cGate * outputgate) {
    table[dest] = outputgate; 
}

void ForwardingTable::remove(NodeAddressT dest) {
    table.erase(dest);
}

cGate * ForwardingTable::lookup(NodeAddressT dest) const {
    DestinationMapT::const_iterator i = table.find(dest);

    if (i == table.end()) {
	return NULL;
    }
    else {
	return i->second;
    }
}

bool ForwardingTable::isNeighbor(NodeAddressT dest) const {
    DestinationMapT::const_iterator i = table.find(dest);
    if (i == table.end()) return false;
    else {
	return (i->second->destinationGate()->ownerModule()->ancestorPar("nodeAddr").longValue() == dest);
    }
}

// void ForwardingTable::printRoute(NodeAddressT dest) {
//     ev.printf(""
// }

// void ForwardingTable::printTable() {
// }
