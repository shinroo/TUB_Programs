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

#include <cassert>
#include <omnetpp.h>
#include "Application.h"
#include "Dijkstra.h"
#include "../support/Topology.h"
#include "../messages/LocalLinkChangeNotif_m.h"

/** Routing daemon that creates its routes by faking, i.e. retrieving the
 *  topology from the simulation core without exchanging routing messages.
 */
class FakedRoutingDaemon : public Application {
public:
    Module_Class_Members(FakedRoutingDaemon,Application,0);

protected:
    /// \name Methods from OMNeT++ framework
    //@{
    virtual int numInitStages() const;
    virtual void initialize(int stage);
    virtual void finish();

    /// Handles LocalLinkChangeNotif and triggers route recalculations
    virtual void handleMessage(cMessage * msg);
    //@}

    static int topoSelectFunction(cModule * mod, void *);

private:
    /// Extracts topology from OMNeT++ and calculates routes with Dijkstra.
    void calculateRoutes();

    Topology * topo; ///< Extracted topology

    std::auto_ptr<std::ofstream>  rtableDump; ///< File for dump of routing table.
};

Define_Module(FakedRoutingDaemon);

int FakedRoutingDaemon::numInitStages() const { return 3; }

// topology nodes are network nodes with NetworkStack and routing daemon
int FakedRoutingDaemon::topoSelectFunction(cModule * mod, void *) {
    mod = mod->submodule("networkStack");
    if (mod == NULL) return 0;

    NetworkStack * netwStack = dynamic_cast<NetworkStack*>(mod);

    return netwStack != NULL && netwStack->lookupApplication(PS_ROUTING_DAEMON) != NULL;
}

void FakedRoutingDaemon::calculateRoutes() {
    cTopology tmpTopo;
    tmpTopo.extractFromNetwork(topoSelectFunction,NULL);

    {
	// configure weights
	for (int i = 0; i < tmpTopo.nodes(); ++i) {
	    cTopology::Node * node = tmpTopo.node(i);
	    NetworkStack * currentStack = dynamic_cast<NetworkStack*>(node->module()->submodule("networkStack"));
	    for (int j = 0; j < node->outLinks(); ++j) {
		cTopology::LinkOut * link = node->out(j);
		link->setWeight(currentStack->getMetric(currentStack->getInternalGate(link->localGate())));
	    }
	}
    }

    topo = new Topology(&tmpTopo);

    // do shortest path
    PathMap routes = doDijkstra(topo,topo->getNodeByAddr(networkStack->getNodeAddr()));
    if (rtableDump.get()) dumpPathMap(*rtableDump,networkStack->getNodeAddr(),routes);

    // Now set routing table
    for (PathMap::iterator i = routes.begin(); i != routes.end(); ++i) {
	networkStack->addRoute(i->first->getNodeAddr(),
			       networkStack->lookupNeighbor(i->second->getDestNode()->getNodeAddr()));
    }
}

void FakedRoutingDaemon::initialize(int stage) {
    switch (stage) {
    case 0: {
	Application::initialize();
	topo = NULL;
	par("applAddr").setLongValue(PS_ROUTING_DAEMON);
	break;
    }
    case 1: break; // here NetworkStacks detect applications and neighbors
    case 2: {
        if (strlen(par("rtableDumpFile").stringValue()) > 0) {
            rtableDump.reset(new std::ofstream(par("rtableDumpFile").stringValue()));
            if (!*rtableDump) throw new cRuntimeError("Could not create %s",par("rtableDumpFile").stringValue());
        }
	calculateRoutes();
	break;
    }
//     case 3: {
// 	NodeAddressT myNodeAddr = ancestorPar("nodeAddr").longValue();
// 	for (int i = 0; i < topo.nodes(); ++i) {
// 	    sTopoNode * node = topo.node(i);
// 	    NetworkStack * otherNetworkStack = dynamic_cast<NetworkStack*>(node->module());
// 	    NodeAddressT addr = otherNetworkStack->ancestorPar("nodeAddr");
// 	    if (addr != myNodeAddr) {
// 		FakedRoutingDaemon * otherRouter = dynamic_cast<FakedRoutingDaemon*>(otherNetworkStack->lookupApplication(FAKED_ROUTING_DAEMON)->destinationGate()->ownerModule());
// 		cGate * outgate = otherRouter->topo.nodeFor(networkStack)->path(0)->localGate();
// 		networkStack->addRoute(addr,outgate);
// 	    }
// 	}
// 	break;
//     }
    default: assert(0);
    }
}


void FakedRoutingDaemon::finish() {
    delete rtableDump.release(); // To flush file here.    
}


void FakedRoutingDaemon::handleMessage(cMessage * msg) {
    LocalLinkChangeNotif * notif = dynamic_cast<LocalLinkChangeNotif*>(msg);
    assert(notif);

    // We dont use the information in the notification here, so delete it
    // right now
    delete notif;
    notif = NULL;

    cTopology fakedRoutingDaemons;
    fakedRoutingDaemons.extractByModuleType("FakedRoutingDaemon",NULL);

    for (int i = 0; i < fakedRoutingDaemons.nodes(); ++i) {
	dynamic_cast<FakedRoutingDaemon*>(fakedRoutingDaemons.node(i)->module())->calculateRoutes();
    }
}
