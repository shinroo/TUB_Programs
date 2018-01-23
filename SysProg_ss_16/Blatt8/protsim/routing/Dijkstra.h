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

#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <fstream>
#include <omnetpp.h>
#include "../support/Topology.h"

/// Type for mapping of nodes to links.
typedef std::map<TopologyNode *,TopologyLink *,CompareTopologyNode> PathMap;

/** 
 * Dijkstra shortest path routing algorithm.
 * 
 * @param topo Topology to calculate shortest path on.
 * @param src Source node within the Topology.
 * 
 * @return Mapping of destination nodes to outgoing links on the source node.
 */
PathMap doDijkstra(Topology * topo, TopologyNode * src);

/// Dump the path map to stream os
void dumpPathMap(std::ostream& os, NodeAddressT node, const PathMap& routes);

#endif /* DIJKSTRA_H */
