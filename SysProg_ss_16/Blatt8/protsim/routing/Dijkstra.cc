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

#include <set>
#include <map>
#include <cassert>
#include "Dijkstra.h"
#include "../common/NetworkStack.h"

PathMap doDijkstra(Topology * topo, TopologyNode * src) {
    typedef std::map<TopologyNode *,double,CompareTopologyNode> DistanceVector;

    Topology::NodeSet N = topo->getNodes(); // set of nodes not handled so far
    Topology::NodeSet M; // set of nodes with are already optimized 
    DistanceVector D; // map of distances to each node
    PathMap P; // map of next hop to the destination

    // Initialize distance vector
    for (Topology::NodeSet::iterator i = N.begin(); i != N.end(); ++i) {
	D[*i] = INFINITY;
    }
    D[src] = 0.0;

    TopologyNode * closestNode = src;
    
    // now do the work
    while (!N.empty()) {
	TopologyNode * current = closestNode;

	if (N.erase(current) == 0) {
	    assert(0);
	}

	M.insert(current);
	
	{
	    for (int i = 0; i < current->getNumLinks(); ++i) {
		TopologyLink * link = current->getLink(i);
		TopologyNode * remote = link->getDestNode();
		if (N.find(remote) != N.end()) {
		    if (D[current] + link->getWeight() < D[remote]) {
			D[remote] = D[current]+link->getWeight();
			if (current == src) {
			    P[remote] = link;
			} else {
			    P[remote] = P[current];
			}
		    }
		}
	    }

	}

	{
	    double minDist = INFINITY;
	    for (Topology::NodeSet::iterator i = N.begin(); i != N.end();
		 ++i) {
		if (D[*i] < minDist) {
		    minDist = D[*i];
		    closestNode = *i;
		}
	    }

	}
    }

    return P;
}


void dumpPathMap(std::ostream& os, NodeAddressT node, const PathMap& routes) {

    os << "rtable node " << node << " time " << simulation.simTime() << "\n";

    for (PathMap::const_iterator i = routes.begin(); i != routes.end(); ++i) {
        
        os << "  " << i->first->getNodeAddr() << " via " << i->second->getDestNode()->getNodeAddr() << "\n";
        
    }

    os << "end" << std::endl;
}
