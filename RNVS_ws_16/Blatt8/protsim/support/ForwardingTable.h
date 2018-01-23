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

#ifndef ROUTING_TABLE_H
#define ROUTING_TABLE_H

#include <map>
#include <omnetpp.h>
#include "protsim_defines.h"

/**
 * Class for all kinds of tables that map node addresses to links.
 * Although called ForwardingTable, it is also used for neighbor and
 * application tables.
 */
class ForwardingTable : public cObject {
public:
    /// \name Overridden methods from cObject
    //@{
    virtual const char * className() const;
    virtual DUP_RETVAL * dup() const;
#ifdef OMNET30
    virtual std::string info();
#else
    virtual void info(char *buf);
#endif
    virtual void writeContents(std::ostream& os) const;
    virtual void writeContents(std::ostream& os) {
        const_cast<const ForwardingTable*>(this)->writeContents(os);
    }

    //@}

    // Routing table specific functionality

    /// Clear the table.
    virtual void clear();

    /** 
     * Insert a node/gate pair into the table.
     * 
     * @param dest Destination node address.
     * @param outputgate Gate towards the destination.
     */
    virtual void add(NodeAddressT dest, cGate * outputgate);

    /** 
     * Removes a node from the table.
     * 
     * @param dest Destination node address.
     */
    virtual void remove(NodeAddressT dest);

    /** 
     * Lookup gate toward destination.
     * 
     * @param dest Destination node address.
     * @return Gate toward destination.
     */
    virtual cGate * lookup(NodeAddressT dest) const;

    /** 
     * Checks whether a node is a direct neighbor.
     * 
     * @param dest Destination node address.
     * @return whether \c dest is a direct neighbor. Only takes routes in this
     *         table into account.
     */
    virtual bool isNeighbor(NodeAddressT dest) const;

    //virtual void printRoute(NodeAddressT dest);
    //virtual void printTable();

private:
    /// Type for the node address/gate mapping
    typedef std::map<NodeAddressT,cGate *> DestinationMapT;

    DestinationMapT table; ///< The table.
};

inline std::ostream& operator<<(std::ostream& os, const ForwardingTable& table) {
    table.writeContents(os);
    return os;
}

#endif /* ROUTING_TABLE_H */
