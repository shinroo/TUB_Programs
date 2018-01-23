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
#include "NetworkStack.h"
#include "ExternalApplication.h"
#include "../messages/NetworkPacket_m.h"
#include "../messages/LocalLinkChangeNotif_m.h"
#include "../messages/LocalPacketNotif_m.h"
#include "../support/Parser.h"

std::istream& operator>>(std::istream& is, cPar& p) {
  std::string s;
  is >> s;
  p.setFromText(s.c_str(),'D');
  return is;
}

Define_Module(NetworkStack);

NetworkStack::~NetworkStack() {
    // in the destructor we can save the setting to NULL
    delete [] virtualQueues;
    delete [] costs;
    delete metricFunc;
}

NodeAddressT NetworkStack::getNodeAddr() const {
    return nodeAddr;
}

void NetworkStack::clearRoutes() {
    routes.clear();
}

void NetworkStack::addRoute(NodeAddressT dest, cGate * outputgate) {
    assert(outputgate);
    assert(strcmp(outputgate->name(),"netOut")==0);

    if (dumpRoutes) routingTableDump.record(dest,lookupNeighbor(outputgate));
    routes.add(dest,outputgate);
}

void NetworkStack::addRoute(NodeAddressT dest, NodeAddressT nextHop) {
    addRoute(dest, lookupNeighbor(nextHop));
}

void NetworkStack::removeRoute(NodeAddressT dest) {
    routes.remove(dest);
}

cGate * NetworkStack::lookupRoute(NodeAddressT dest) const {
    return routes.lookup(dest);
}

bool NetworkStack::isNeighbor(NodeAddressT dest) const {
    return neighbors.isNeighbor(dest);
}


cGate * NetworkStack::getExternalGate(cGate * outgate) const {
    // parentModule must be the network node
    // gates with identical name and index of node and network stack must
    //     must correspond to each other
    cGate * retval = parentModule()->gate(outgate->name(),outgate->index());

    assert(retval);
    return retval;
}

cBasicChannel * NetworkStack::getExternalLink(cGate * outgate) const {
    // parentModule must be the network node
    // gates with identical name and index of node and network stack must
    //     must correspond to each other
    cBasicChannel * link = dynamic_cast<cBasicChannel*>(getExternalGate(outgate)->channel());
    assert(link);
    return link;
}

cGate * NetworkStack::getInternalGate(cGate * outgate) {
    // parentModule must be the network node
    // gates with identical name and index of node and network stack must
    //     must correspond to each other
    cGate * retval = gate(outgate->name(),outgate->index());
    assert(retval);
    return retval;
}


NodeAddressT NetworkStack::lookupNeighbor(cGate * outgate) const {
    assert(outgate);
    assert(strcmp(outgate->name(),"netOut")==0);

    return getExternalGate(outgate)->destinationGate()->ownerModule()->ancestorPar("nodeAddr").longValue();
}

cGate * NetworkStack::lookupNeighbor(NodeAddressT peer) const {
    return neighbors.lookup(peer);
}

int NetworkStack::getNumNeighbors() const {
    return gate("netOut",0)->size();
}

const cGate * NetworkStack::lookupNeighborAt(int index) const {
    return gate("netOut",index);
}

cGate * NetworkStack::lookupNeighborAt(int index) {
    return gate("netOut",index);
}

double NetworkStack::getDelayToNeighbor(cGate * outgate) const {
    assert(outgate);
    assert(strcmp(outgate->name(),"netOut")==0);

#ifdef OMNET30
    return getExternalLink(outgate)->delay();
#else
    return getExternalLink(outgate)->delay()->doubleValue();
#endif
}

double NetworkStack::getDelayToNeighbor(NodeAddressT peer) const {
    return getDelayToNeighbor(lookupNeighbor(peer));
}

double NetworkStack::getDatarateToNeighbor(cGate * outgate) const {
    assert(outgate);
    assert(strcmp(outgate->name(),"netOut")==0);

#ifdef OMNET30
    return getExternalLink(outgate)->datarate();
#else
    return getExternalLink(outgate)->datarate()->doubleValue();
#endif
}

double NetworkStack::getDatarateToNeighbor(NodeAddressT peer) const {
    return getDatarateToNeighbor(lookupNeighbor(peer));
}

double NetworkStack::getDynamicDelayToNeighbor(cGate * outgate) const {
    assert(outgate);
    assert(strcmp(outgate->name(),"netOut")==0);

    return getDelayToNeighbor(outgate) +
	max(getExternalGate(outgate)->transmissionFinishes()-simTime(),0.0);
}

double NetworkStack::getDynamicDelayToNeighbor(NodeAddressT peer) const {
    return getDynamicDelayToNeighbor(lookupNeighbor(peer));
}

double NetworkStack::getVirtualQueue(cGate * outgate) const {
    assert(outgate);
    assert(strcmp(outgate->name(),"netOut")==0);

    return max((getExternalGate(outgate)->transmissionFinishes()-simTime())*
	       getDatarateToNeighbor(outgate),0.0);
}

double NetworkStack::getVirtualQueue(NodeAddressT peer) const {
    return getVirtualQueue(lookupNeighbor(peer));
}

double NetworkStack::getMetric(cGate * outgate) const {
    assert(outgate);
    assert(strcmp(outgate->name(),"netOut")==0);

    if (metricFunc == NULL) return 1.0; // default is hops
    else return metricFunc->getMetric(this,outgate);
}

double NetworkStack::getMetric(NodeAddressT peer) const {
    return getMetric(lookupNeighbor(peer));
}

cGate * NetworkStack::lookupApplication(ApplicationAddressT dest) const {
    return applications.lookup(dest);
}

void NetworkStack::setMetricFunc(const char * funcName) {
    delete metricFunc;
    metricFunc = static_cast<MetricFunc*>(createOne(funcName));
    assert(metricFunc);
}

const char * NetworkStack::getMetricFunc() const {
    if (metricFunc == NULL) return NULL;
    else return metricFunc->className();
}

int NetworkStack::numInitStages() const {
    return 2;
}

void NetworkStack::initialize(int stage) {
    switch (stage) {
    case 0: {
	// Cache our node address
	nodeAddr = ancestorPar("nodeAddr").longValue();

	// Register with scheduler
	ProtSimSocketRTScheduler * scheduler = dynamic_cast<ProtSimSocketRTScheduler *>(simulation.scheduler());
	if (scheduler) {
	    scheduler->registerNode(nodeAddr, this);
	    externalApplType = ancestorPar("externalApplType").stringValue();
            externalGenerateTransport = ancestorPar("externalGenerateTransport").boolValue();
	    callFinishExternalAppl = ancestorPar("callFinishExternalAppl").boolValue();
	}

	// Set metric
	if (par("metric").stringValue()) {
	    setMetricFunc(par("metric").stringValue());
	}

        dumpRoutes = par("dumpRoutes").boolValue();

        // Set cost parameters of links
	char c[120];
	costs = new double[gate("netOut",0)->size()];
	for (int i = 0; i < gate("netOut",0)->size(); ++i) {
	    cPar * newPar = &(getExternalLink(gate("netOut",i))->addPar("cost"));

#ifdef OMNET30
	    snprintf(c,sizeof(c),"%s.netOut[%d].cost",fullPath().c_str(),i);
#else
	    snprintf(c,sizeof(c),"%s.netOut[%d].cost",fullPath(),i);
#endif
#ifdef OMNET31
	    newPar->setFromText(ev.getParameter(simulation.runNumber(),c).c_str(),'D');
#else
	    newPar->setFromText(ev.getParameter(simulation.runNumber(),c),'D');
#endif
	    //costs[i] = newPar->doubleValue();
	    snprintf(c,sizeof(c),"cost[%d]",i);
#ifdef OMNET32
	    //createWatch_genericAssignable(c,*newPar);
	    cPar * modPar = addPar(c);
	    modPar->setDoubleValue(newPar->doubleValue());
#else
	    new cWatch(c,costs[i]);
#endif
	}

        WATCH(routes);
        WATCH(neighbors);
        WATCH(applications);

	// set maxTTL
	maxTTL = simulation.systemModule()->par("maxTTL").longValue();
	break;

	// set names
	routes.setName("forwarding table");
	neighbors.setName("neighbor table");
	applications.setName("application table");
    }
    case 1: {
	// autodetect neighbors
	virtualQueues = new double[gate("netOut",0)->size()];
	{
	    for (int i = 0; i < gate("netOut",0)->size(); ++i) {
		cGate * outgate = gate("netOut",i);
		neighbors.add(lookupNeighbor(outgate), outgate);
		virtualQueues[i] = 0.0;
		char c[20];
		snprintf(c,sizeof(c),"virtualQueues[%d]",i);
#ifdef OMNET32
		createWatch(c,virtualQueues[i]);
#else
		new cWatch(c,virtualQueues[i]);
#endif
	    }
	}

	// autodetect applications
	{
	    for (int i = 0; i < gate("toAppl",0)->size(); ++i) {
		cGate * outgate = gate("toAppl",i);
		long applAddr = outgate->toGate()->ownerModule()->par("applAddr").longValue();
		if (applAddr != PS_DUMMY_APP) applications.add(applAddr, outgate);
	    }
	}
	break;
    }
    default: assert(0);
    }
}

void NetworkStack::finish() {
    delete [] virtualQueues;
    virtualQueues = NULL;
    delete [] costs;
    costs = NULL;
    delete metricFunc;
    metricFunc = NULL;
}

void NetworkStack::handleMessage(cMessage * msg) {
    assert(msg->kind() == eNETWORK_PACKET ||
	   msg->kind() == eLOCAL_MESSAGE);

    NetworkPacket * packet = dynamic_cast<NetworkPacket*>(msg);
    assert(packet);
    bool local = (msg->kind() == eLOCAL_MESSAGE);
    bool locally_originated = false;

    // Insert source application and node for locally generated packets
    if (packet->arrivalGate() && strcmp(packet->arrivalGate()->name(),"fromAppl")==0) {
	locally_originated = true;


	// Leave valid source nodes of alert packets untouched
	if (!packet->getAlert() || packet->getSrcNode() == PS_NODE_UNKNOWN) {
	    packet->setSrcNode(nodeAddr);
	    // reset TTL
	    packet->setTTL(0);
	}

	// The same with applications
	if (!packet->getAlert() || packet->getSrcAppl() == PS_APP_UNKNOWN)
	    packet->setSrcAppl(packet->arrivalGate()->fromGate()->ownerModule()->par("applAddr").longValue());
    }

    // Deliver locally if this is a local message or the alert flag is set
    // or we are the destination node
    if (local || (packet->getAlert() && !locally_originated) ||
	packet->getDestNode() == nodeAddr) {

	deliverLocally(packet);
    }
    // No local message, no alert flag: Packet should be forwarded
    else {
	sendOut(packet);
    }
}

void NetworkStack::deliverLocally(NetworkPacket * packet) {
    cGate * local_out = lookupApplication(packet->getDestAppl());
    if (local_out == NULL) {
	// drop packet
	ev << "No application "<< packet->getDestAppl()
	   << ": Message " << packet->name() << " dropped in node "
	   << nodeAddr << endl;
	delete packet;
	packet = NULL;
    } else {
	// Invalidate destination node if the packet is for us
	// This is for the case where the application forgets to set a
	// new destination when resending the message
	if (packet->getDestNode() == nodeAddr) {
	    packet->setDestNode(PS_NODE_UNKNOWN);
	    packet->setDestAppl(PS_APP_UNKNOWN);
	}

	send(packet, local_out);
    }
}

void NetworkStack::sendOut(NetworkPacket * packet) {
    if (packet->getTTL() >= maxTTL) {
	ev << "TTL exceeded: Packet " << packet->name()
	   << " dropped in node " << nodeAddr << endl;
	delete packet;
	packet = NULL;
	return;
    }

    // No local message, no alert flag, TTL okay
    switch (packet->getDestNode()) {
    case PS_NODE_UNKNOWN: { // Drop packets with no destination set
	ev << "No destination set: Packet " << packet->name()
	   << " dropped in node " << nodeAddr << endl;
	delete packet;
	packet = NULL;
	break;
    }
    case PS_DEST_BROADCAST_ALL:
    case PS_DEST_BROADCAST_LIMIT: {
	// Save the last hop for later evaluation
	NodeAddressT lastNode = packet->getLastNode();
	// Set this node as the last hop
	packet->setLastNode(nodeAddr);
	packet->setTTL(packet->getTTL()+1);

	for (int i = 0; i < getNumNeighbors(); ++i) {
	    bool sent = false;
	    cGate * output = lookupNeighborAt(i);

	    // Only send if sending to all neighbors is requested or
	    // the neighbor is not the one the message originally came
	    // from
	    if (packet->getDestNode() == PS_DEST_BROADCAST_ALL ||
		lookupNeighbor(output) != lastNode) {

		// Each packet may only be sent once, so save a copy
		NetworkPacket * copy = dynamic_cast<NetworkPacket*>(packet->dup());
		send(packet,output);
		sent = true;
		packet = copy;
	    }
	}

	// Always delete as we duplicated the packet before last sending
	// This may also be the original packet for limited broadcast and
	// one interface
	delete packet;
	packet = NULL;
	break;
    }
    default: { // The destination is remote
	cGate * output = lookupRoute(packet->getDestNode());

	// No route
	if (output != NULL) { // send it out
	    packet->setLastNode(nodeAddr);
	    packet->setTTL(packet->getTTL()+1);
	    send(packet,output);
	    virtualQueues[output->index()] = getVirtualQueue(output);
	    assert(output->toGate()->datarate() == NULL ||
		   output->toGate()->datarate()->doubleValue() == 0.0 ||
		   virtualQueues[output->index()] != 0.0);

	}
	// drop packet
	else {
	    ev << "No route to host "<< packet->getDestNode()
	       << ": Packet " << packet->name() << " dropped in node "
	       << nodeAddr << endl;
	    delete packet;
	    packet = NULL;
	}
    }
    }
}

void NetworkStack::handleParameterChange(const char * parname) {
    if (strcmp(parname,"shell") == 0 &&
        strcmp(par("shell").stringValue(),"") != 0) {
	shell(par("shell").stringValue(),&par("shell_result"));
	par("shell").setStringValue("");
    }
    else if (strncmp(parname,"cost[",5) == 0) {
        int index;
        sscanf(parname,"cost[%d]",&index);
	double value = par(parname).doubleValue();
 	cPar * myPar = &getExternalLink(gate("netOut",index))->par("cost");
	double oldval = myPar->doubleValue();
	myPar->setDoubleValue(value);

	// send local notification
	LocalLinkChangeNotif * notif = new LocalLinkChangeNotif("LocalLinkChange",eLOCAL_MESSAGE);
	notif->setBaseName("LocalLinkChange");
	notif->setId(eLOCAL_LINK_CHANGE_NOTIFICATION);
	notif->setSrcAppl(-1);
	notif->setDestAppl(PS_ROUTING_DAEMON);
	notif->setPriority(-1);
	notif->setLinkIndex(index);
	notif->setNextHop(lookupNeighbor(lookupNeighborAt(index)));
	notif->setOldCost(oldval);
	notif->setNewCost(value);
	scheduleAt(simTime(),notif);
    }
}

void NetworkStack::sendLocalPacketNotif(NetworkPacket * packet, bool noroute) {
    LocalPacketNotif * notif = new LocalPacketNotif("LocalPacketNotif",eLOCAL_MESSAGE);
    notif->setBaseName("LocalPacketNotif");
    notif->setId(eLOCAL_PACKET_NOTIFICATION);
    notif->setSrcAppl(-1);
    notif->setDestAppl(PS_ROUTING_DAEMON);
    notif->setPriority(-1);
    notif->setNoRoute(noroute);
    notif->encapsulate(dynamic_cast<cMessage*>(packet->dup()));
    scheduleAt(simTime(),notif);
}

void NetworkStack::shell(const char * command, cPar * result) {
    enum Token { eCONNECTION_PAR=1 };

    Parser::Translation transtable[] = {
	{ "conpar", eCONNECTION_PAR, 3, 3 },
	{ NULL, 0, 0, 0 }
    };

    try {
	StringParser parser(command, transtable);

	while (parser.nextLine() > 0) {
	    switch (parser.getToken()) {
	    case eCONNECTION_PAR:
	    {
		string parName;
		unsigned index;
		double value;

		parser.getString(0,&parName);
		parser.getUInt(1,&index);
		parser.getDouble(2,&value);

		if (index < (unsigned)gate("netOut",0)->size()) {
                    cChannel * channel = getExternalLink(gate("netOut",index));
		    cPar * myPar;
		    double oldval;
		    if (channel->hasPar(parName.c_str())) {
			myPar = &channel->par(parName.c_str());
			oldval = myPar->doubleValue();
		    } else {
			myPar = &channel->addPar(parName.c_str());
			oldval = 1.0;
		    }
		    myPar->setDoubleValue(value);

		    if (parName == string("cost")) {
			costs[index] = value;

			// send local notification
			LocalLinkChangeNotif * notif = new LocalLinkChangeNotif("LocalLinkChange",eLOCAL_MESSAGE);
			notif->setBaseName("LocalLinkChange");
			notif->setId(eLOCAL_LINK_CHANGE_NOTIFICATION);
			notif->setSrcAppl(-1);
			notif->setDestAppl(PS_ROUTING_DAEMON);
			notif->setPriority(-1);
			notif->setLinkIndex(index);
			notif->setNextHop(lookupNeighbor(lookupNeighborAt(index)));
			notif->setOldCost(oldval);
			notif->setNewCost(value);
			scheduleAt(simTime(),notif);
		    }

		    if (result) result->setStringValue("okay");
		}
		else {
		    if (result) result->setStringValue("out of bounds");
		}
		break;
	    }
	    default:
		// Invalid keywords are checked in nextLine(). We have an internal error if we reach this.
		assert(0);
		break;
	    }
	}
	parser.close();
    }
    catch (const Parser::Error& e) {
	if (result) result->setStringValue(e.what());
    }
}

void NetworkStack::createApplication(ProtSimSocketRTScheduler::ApplicationDesc* appl) {
    if (lookupApplication(appl->applAddr) != NULL) {
	throw new cRuntimeError("NetworkStack: Application already exists");
    }

    // find factory object
    cModuleType *moduleType = findModuleType(externalGenerateTransport ?
                                             "ApplicationScope" : externalApplType.c_str());

    in_addr ipAddr;
    ipAddr.s_addr = htonl(appl->ipAddr);
    char modName[40];
    snprintf(modName, sizeof(modName), "%ld:%s:%d", appl->applAddr, inet_ntoa(ipAddr), appl->port);

    // create
    cModule *module = moduleType->create(modName, parentModule());

    // create and connect gates
    char fromApplGateName[32];
    char toApplGateName[32];
    snprintf(fromApplGateName, sizeof(fromApplGateName), "fromExternalAppl%ld", appl->applAddr);
    snprintf(toApplGateName, sizeof(toApplGateName), "toExternalAppl%ld", appl->applAddr);
    cGate* fromAppl = gate(fromApplGateName);
    cGate* toAppl = gate(toApplGateName);

    if (fromAppl == NULL) {
	assert(!toAppl);
	fromAppl = addGate(fromApplGateName, 'I');
	toAppl = addGate(toApplGateName, 'O');
    }
    assert(toAppl);
    toAppl->connectTo(module->gate("in"), new cBasicChannel());
    module->gate("out")->connectTo(fromAppl, new cBasicChannel());

    if (externalGenerateTransport) {
        module->par("applAddr") = appl->applAddr;
        module->par("applType") = externalApplType.c_str();
    }

    // create internals, and schedule it
    module->buildInside();
    module->scheduleStart( simTime() );

    if (externalGenerateTransport) {
        module = module->submodule("userApplication");
    }
    appl->module = check_and_cast<ExternalApplication*>(module);
    
    // set up parameters
    module->par("applAddr") = appl->applAddr;
    if (!module->hasPar("ipAddr")) module->addPar("ipAddr");
    if (!module->hasPar("port")) module->addPar("port");
    module->par("ipAddr") = appl->ipAddr;
    module->par("port") = appl->port;
    
    // register application in application table
    applications.add(appl->applAddr, toAppl);
    module->callInitialize();
}

void NetworkStack::deleteApplication(ProtSimSocketRTScheduler::ApplicationDesc* appl, bool endRun) {
    if (lookupApplication(appl->applAddr) == NULL) {
	throw new cRuntimeError("NetworkStack: Application not registered");
    }

    cModule* module = externalGenerateTransport ? appl->module->parentModule() : appl->module;
    appl->module = NULL;
    
    if (endRun || callFinishExternalAppl) module->callFinish();

    // deregister application
    applications.remove(appl->applAddr);

    // disconnect gates
    char toApplGateName[32];
    snprintf(toApplGateName, sizeof(toApplGateName), "toExternalAppl%ld", appl->applAddr);
    gate(toApplGateName)->disconnect();
    module->gate("out")->disconnect();

    // remove module
    module->deleteModule();
}
